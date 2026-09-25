// Tubería de video del Cerebro P4 (v2, en paralelo). Tres tareas, cada una con su motor:
//   [dec]  JPEG del PC --decodificador JPEG HW--> cuadro RGB565 1920x1080 (2 búferes que se turnan)
//   [out]  IA: copia 640x360 para los detectores (CPU) · auto-encuadre: recorte+zoom (PPA SRM)
//          cuadros de detección (PPA relleno) · barra de información (PPA mezcla alfa)
//          --codificador JPEG HW--> webcam MJPEG + dashboard
//   [h264] cada tanto: RGB565 -> YUV420 (PPA SRM) --codificador H.264 HW (ROI en caras, vectores de
//          movimiento)--> repetición de los últimos segundos + webcam H.264
// Medido en este chip (v1.0, PSRAM a 200 MHz): la memoria externa es el cuello (~300 MB/s entre todos).
// La versión anterior (secuencial, hasta 2560x1440) quedó en docs/versiones/pipeline_v1_secuencial.c
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/jpeg_decode.h"
#include "driver/jpeg_encode.h"
#include "driver/ppa.h"
#include "esp_async_color_convert.h"
#include "esp_h264_enc_single_hw.h"
#include "esp_h264_enc_single.h"
#include "esp_h264_enc_param.h"
#include "esp_h264_enc_param_hw.h"
#include "esp_h264_alloc.h"
#include "app.h"

static const char *TAG = "video";

#define W 1920
#define H 1080
#define H16 1088

// ---------- motores ----------
static jpeg_decoder_handle_t s_jdec;
static jpeg_encoder_handle_t s_jenc;
static ppa_client_handle_t   s_srm, s_fill, s_blend, s_srm_yuv;
static esp_h264_enc_handle_t s_h264;
static esp_h264_enc_param_hw_handle_t s_h264p;
static int  s_h264_kbps;
static bool s_jenc_422;

// ---------- cuadros ----------
#define N_FB 2
typedef struct {
    uint8_t *buf;
    int w, h;
    bool real;          // viene de la cámara (no es el patrón de prueba)
    bool publish;       // se muestra (los cuadros de relleno del modo máximo no)
    int64_t t_rx;
} frame_t;
static frame_t s_fb[N_FB];
static size_t s_fb_cap;
static QueueHandle_t s_free_q, s_ready_q;

static uint8_t *s_comp; static size_t s_comp_cap;     // salida con zoom
static uint8_t *s_yuv;  static size_t s_yuv_cap;      // entrada del H.264
static uint8_t *s_bench_jpeg; static size_t s_bench_len;

#define N_JOUT 2
static uint8_t *s_jout[N_JOUT];
static size_t   s_jout_cap, s_jout_len[N_JOUT];
static int      s_jlatest = -1, s_jusb = -1;
static volatile uint32_t s_jseq;
static SemaphoreHandle_t s_jmux;
static volatile int64_t s_web_view_t;                 // último pedido del dashboard

static uint8_t *s_hout; static size_t s_hout_cap;
static esp_h264_enc_mv_data_t *s_mv; static uint32_t s_mv_cap;
static volatile float s_motion_pct;
static SemaphoreHandle_t s_yuv_ready, s_yuv_free, s_srm_done;
static volatile bool s_uvc_h264_synced;
static det_list_t s_roi_dets;
static int s_roi_cx, s_roi_cy, s_roi_ox, s_roi_oy;
static float s_roi_s;

// ---------- repetición: anillo de cuadros H.264 ----------
#define RING_CAP (3 * 1024 * 1024)
#define RF_MAX   2048
typedef struct { uint32_t off, len, id; int64_t t; uint8_t idr; } rframe_t;
static uint8_t  *s_ring;
static uint32_t  s_ring_head;
static rframe_t *s_rf;
static int       s_rf_head, s_rf_count;
static uint32_t  s_rf_id;
static SemaphoreHandle_t s_ring_mux;

// ---------- auto-encuadre ----------
static float   s_cx = W / 2, s_cy = H / 2, s_cw = W;
static int64_t s_last_face_t;

static void *alloc64(size_t n)
{
    return heap_caps_aligned_calloc(64, 1, P4_ALIGN(n, 64), MALLOC_CAP_SPIRAM);
}

// =============================== ayudantes PPA ===============================
static bool srm(ppa_client_handle_t cli, const void *in, int in_w, int in_h, int bx, int by, int bw, int bh,
                ppa_srm_color_mode_t in_cm, void *out, size_t out_cap, int out_w, int out_h, int ox, int oy,
                ppa_srm_color_mode_t out_cm, float scale, bool mirror)
{
    ppa_srm_oper_config_t op = {
        .in = { .buffer = in, .pic_w = in_w, .pic_h = in_h, .block_w = bw, .block_h = bh,
                .block_offset_x = bx, .block_offset_y = by, .srm_cm = in_cm },
        .out = { .buffer = out, .buffer_size = out_cap, .pic_w = out_w, .pic_h = out_h,
                 .block_offset_x = ox, .block_offset_y = oy, .srm_cm = out_cm },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
        .scale_x = scale, .scale_y = scale,
        .mirror_x = mirror,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };
    int64_t t0 = esp_timer_get_time();
    esp_err_t r = ppa_do_scale_rotate_mirror(cli, &op);
    stats_busy(ENG_PPA, t0, esp_timer_get_time());
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "PPA SRM falló (%s)", esp_err_to_name(r));
    }
    return r == ESP_OK;
}

static void fill_rect(uint8_t *dst, size_t cap, int x, int y, int w, int h, uint32_t argb)
{
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > W) w = W - x;
    if (y + h > H) h = H - y;
    if (w <= 0 || h <= 0) return;
    ppa_fill_oper_config_t op = {
        .out = { .buffer = dst, .buffer_size = cap, .pic_w = W, .pic_h = H,
                 .block_offset_x = x, .block_offset_y = y, .fill_cm = PPA_FILL_COLOR_MODE_RGB565 },
        .fill_block_w = w, .fill_block_h = h,
        .fill_argb_color = { .val = argb },
        .mode = PPA_TRANS_MODE_BLOCKING,
    };
    int64_t t0 = esp_timer_get_time();
    ppa_do_fill(s_fill, &op);
    stats_busy(ENG_PPA, t0, esp_timer_get_time());
}

static void box(uint8_t *dst, size_t cap, int x0, int y0, int x1, int y1, uint32_t argb, int t)
{
    fill_rect(dst, cap, x0, y0, x1 - x0, t, argb);
    fill_rect(dst, cap, x0, y1 - t, x1 - x0, t, argb);
    fill_rect(dst, cap, x0, y0, t, y1 - y0, argb);
    fill_rect(dst, cap, x1 - t, y0, t, y1 - y0, argb);
}

static void blend_hud(uint8_t *dst, size_t cap)
{
    int hw, hh;
    uint8_t *hud = hud_buf(&hw, &hh);
    if (!hud || hw != W) return;
    ppa_blend_oper_config_t op = {
        .in_bg = { .buffer = dst, .pic_w = W, .pic_h = H, .block_w = W, .block_h = hh,
                   .blend_cm = PPA_BLEND_COLOR_MODE_RGB565 },
        .in_fg = { .buffer = hud, .pic_w = hw, .pic_h = hh, .block_w = hw, .block_h = hh,
                   .blend_cm = PPA_BLEND_COLOR_MODE_ARGB8888 },
        .out = { .buffer = dst, .buffer_size = cap, .pic_w = W, .pic_h = H,
                 .blend_cm = PPA_BLEND_COLOR_MODE_RGB565 },
        .bg_alpha_update_mode = PPA_ALPHA_NO_CHANGE,
        .fg_alpha_update_mode = PPA_ALPHA_NO_CHANGE,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };
    int64_t t0 = esp_timer_get_time();
    ppa_do_blend(s_blend, &op);
    stats_busy(ENG_PPA, t0, esp_timer_get_time());
}

static bool on_yuv_done(ppa_client_handle_t c, ppa_event_data_t *e, void *u)
{
    BaseType_t hp = pdFALSE;
    xSemaphoreGiveFromISR(s_srm_done, &hp);
    return hp == pdTRUE;
}

// conversión RGB565 -> YUV420 para el H.264 SIN bloquear: corre en el PPA mientras el JPEG codifica
static bool yuv_start(const uint8_t *src)
{
    ppa_srm_oper_config_t op = {
        .in = { .buffer = src, .pic_w = W, .pic_h = H, .block_w = W, .block_h = H, .srm_cm = PPA_SRM_COLOR_MODE_RGB565 },
        .out = { .buffer = s_yuv, .buffer_size = s_yuv_cap, .pic_w = W, .pic_h = H, .srm_cm = PPA_SRM_COLOR_MODE_YUV420 },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
        .scale_x = 1.0f, .scale_y = 1.0f,
        .mode = PPA_TRANS_MODE_NON_BLOCKING,
    };
    return ppa_do_scale_rotate_mirror(s_srm_yuv, &op) == ESP_OK;
}

// =============================== repetición ===============================
static void ring_append(const uint8_t *p, uint32_t len, bool idr, uint8_t **stored)
{
    *stored = NULL;
    if (len == 0 || len > RING_CAP / 4) return;
    xSemaphoreTake(s_ring_mux, portMAX_DELAY);
    uint32_t off = s_ring_head;
    if (off + len > RING_CAP) off = 0;
    while (s_rf_count > 0) {
        rframe_t *old = &s_rf[(s_rf_head - s_rf_count + RF_MAX) % RF_MAX];
        bool overlap = old->off < off + len && off < old->off + old->len;
        if (!overlap && s_rf_count < RF_MAX) break;
        s_rf_count--;
    }
    memcpy(s_ring + off, p, len);
    s_rf[s_rf_head] = (rframe_t){ .off = off, .len = len, .id = s_rf_id++, .t = esp_timer_get_time(), .idr = idr };
    s_rf_head = (s_rf_head + 1) % RF_MAX;
    s_rf_count++;
    s_ring_head = off + len;
    *stored = s_ring + off;
    xSemaphoreGive(s_ring_mux);
}

int replay_stream(int (*emit)(void *ctx, const uint8_t *p, size_t n), void *ctx, int seconds)
{
    uint8_t *tmp = heap_caps_malloc(RING_CAP / 4, MALLOC_CAP_SPIRAM);
    if (!tmp) return -1;
    int64_t since = esp_timer_get_time() - (int64_t)seconds * 1000000;
    xSemaphoreTake(s_ring_mux, portMAX_DELAY);
    uint32_t start = UINT32_MAX, first_idr = UINT32_MAX, last = s_rf_id;
    for (int i = s_rf_count; i > 0; i--) {
        rframe_t *f = &s_rf[(s_rf_head - i + RF_MAX) % RF_MAX];
        if (!f->idr) continue;
        if (first_idr == UINT32_MAX) first_idr = f->id;
        if (f->t <= since) start = f->id;
    }
    if (start == UINT32_MAX) start = first_idr;
    xSemaphoreGive(s_ring_mux);
    int total = 0;
    if (start == UINT32_MAX) { free(tmp); return 0; }
    for (uint32_t id = start; id < last; id++) {
        xSemaphoreTake(s_ring_mux, portMAX_DELAY);
        uint32_t age = s_rf_id - id;
        if (age == 0 || age > (uint32_t)s_rf_count) { xSemaphoreGive(s_ring_mux); break; }
        rframe_t *f = &s_rf[(s_rf_head - (int)age + RF_MAX) % RF_MAX];
        uint32_t n = f->len;
        memcpy(tmp, s_ring + f->off, n);
        xSemaphoreGive(s_ring_mux);
        if (emit(ctx, tmp, n) < 0) break;
        total += n;
    }
    free(tmp);
    return total;
}

float pipeline_motion_pct(void) { return s_motion_pct; }
void pipeline_snapshot_request(void) {}

// =============================== H.264 ===============================
void pipeline_early_init(void)
{
    // el H.264 pide ~135 KB de RAM interna contigua: se crea antes que todo lo demás
    esp_h264_enc_cfg_hw_t cfg = {
        .pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY,
        .gop = 15,
        .fps = 30,
        .res = { .width = W, .height = H },
        .rc = { .bitrate = g_cfg.h264_kbps * 1000, .qp_min = 20, .qp_max = 40 },
    };
    if (esp_h264_enc_hw_new(&cfg, &s_h264) != ESP_H264_ERR_OK || esp_h264_enc_open(s_h264) != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "no se pudo crear el H.264 %dx%d", W, H);
        s_h264 = NULL;
        return;
    }
    esp_h264_enc_hw_get_param_hd(s_h264, &s_h264p);
    esp_h264_enc_roi_cfg_t roi = { .roi_mode = ESP_H264_ROI_MODE_DELTA_QP, .none_roi_delta_qp = 3 };
    esp_h264_enc_hw_cfg_roi(s_h264p, roi);
    esp_h264_enc_mv_cfg_t mv = { .mv_mode = ESP_H264_MVM_MODE_P16X16, .mv_fmt = ESP_H264_MVM_FMT_PART };
    esp_h264_enc_hw_cfg_mv(s_h264p, mv);
    s_mv_cap = (W / 16) * (H16 / 16) * sizeof(esp_h264_enc_mv_data_t);
    s_mv = heap_caps_aligned_calloc(64, 1, P4_ALIGN(s_mv_cap, 64), MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    s_h264_kbps = g_cfg.h264_kbps;
    ESP_LOGI(TAG, "H.264 %dx%d listo (ROI en caras + vectores de movimiento)", W, H);
}

static void h264_set_roi(void)
{
    int mbw = W / 16, mbh = H16 / 16, k = 0;
    const det_list_t *dl = &s_roi_dets;
    for (int i = 0; i < dl->n && k < 8; i++) {
        if (dl->d[i].cls != 0) continue;
        int x0 = (int)(((dl->d[i].x0 * W / 10000) - s_roi_cx) * s_roi_s) + s_roi_ox;
        int y0 = (int)(((dl->d[i].y0 * H / 10000) - s_roi_cy) * s_roi_s) + s_roi_oy;
        int x1 = (int)(((dl->d[i].x1 * W / 10000) - s_roi_cx) * s_roi_s) + s_roi_ox;
        int y1 = (int)(((dl->d[i].y1 * H / 10000) - s_roi_cy) * s_roi_s) + s_roi_oy;
        if (g_cfg.mirror) { int a = W - x1, b = W - x0; x0 = a; x1 = b; }
        int mx = x0 / 16, my = y0 / 16, lx = (x1 - x0) / 16 + 1, ly = (y1 - y0) / 16 + 1;
        if (mx < 0) { lx += mx; mx = 0; }
        if (my < 0) { ly += my; my = 0; }
        if (mx + lx > mbw) lx = mbw - mx;
        if (my + ly > mbh) ly = mbh - my;
        if (lx <= 0 || ly <= 0) continue;
        esp_h264_enc_roi_reg_t r = { .x = mx, .y = my, .len_x = lx, .len_y = ly, .qp = -6, .reg_idx = k++ };
        esp_h264_enc_hw_set_roi_region(s_h264p, r);
    }
    for (; k < 8; k++) {
        esp_h264_enc_roi_reg_t r = { .reg_idx = k };
        esp_h264_enc_hw_set_roi_region(s_h264p, r);
    }
}

static void h264_task(void *arg)
{
    while (1) {
        xSemaphoreTake(s_yuv_ready, portMAX_DELAY);
        if (s_h264_kbps != g_cfg.h264_kbps) {
            esp_h264_enc_set_bitrate((esp_h264_enc_param_handle_t)s_h264p, g_cfg.h264_kbps * 1000);
            s_h264_kbps = g_cfg.h264_kbps;
        }
        h264_set_roi();
        esp_h264_enc_in_frame_t in = { .raw_data = { .buffer = s_yuv, .len = W * H16 * 3 / 2 },
                                       .pts = (uint32_t)(esp_timer_get_time() / 1000) };
        esp_h264_enc_out_frame_t out = { .raw_data = { .buffer = s_hout, .len = s_hout_cap } };
        esp_h264_enc_mvm_pkt_t pkt = { .data = s_mv, .len = s_mv_cap };
        esp_h264_enc_hw_set_mv_pkt(s_h264p, pkt);
        int64_t t0 = esp_timer_get_time();
        esp_h264_err_t r = esp_h264_enc_process(s_h264, &in, &out);
        stats_busy(ENG_H264, t0, esp_timer_get_time());
        xSemaphoreGive(s_yuv_free);
        if (r != ESP_H264_ERR_OK) {
            ESP_LOGW(TAG, "H.264 falló (%d)", r);
            continue;
        }
        uint32_t mvlen = 0;
        esp_h264_enc_hw_get_mv_data_len(s_h264p, &mvlen);
        float m = 100.0f * (float)(mvlen / sizeof(esp_h264_enc_mv_data_t)) / (float)((W / 16) * (H16 / 16));
        s_motion_pct = s_motion_pct * 0.7f + m * 0.3f;

        bool idr = out.frame_type == ESP_H264_FRAME_TYPE_IDR;
        uint8_t *stored = NULL;
        ring_append(out.raw_data.buffer, out.length, idr, &stored);
        if (uvc_format() == 2 && uvc_streaming()) {
            if (idr) s_uvc_h264_synced = true;
            if (s_uvc_h264_synced && stored) {
                if (!uvc_send(stored, out.length)) s_uvc_h264_synced = false;   // se esperará el próximo IDR
            }
        } else {
            s_uvc_h264_synced = false;
        }
    }
}

// =============================== JPEG ===============================
static void jpeg_publish(const uint8_t *src, bool publish)
{
    int k = (s_jlatest + 1) % N_JOUT;
    if (k == s_jusb && !uvc_ready()) k = s_jlatest < 0 ? 0 : s_jlatest;   // el otro está viajando por USB
    jpeg_encode_cfg_t cfg = {
        .height = H, .width = W,
        .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
        .sub_sample = s_jenc_422 ? JPEG_DOWN_SAMPLING_YUV422 : JPEG_DOWN_SAMPLING_YUV420,
        .image_quality = g_cfg.jpeg_q,
    };
    uint32_t len = 0;
    int64_t t0 = esp_timer_get_time();
    esp_err_t r = jpeg_encoder_process(s_jenc, &cfg, src, W * H * 2, s_jout[k], s_jout_cap, &len);
    stats_busy(ENG_JENC, t0, esp_timer_get_time());
    if (r != ESP_OK) {
        if (!s_jenc_422) {
            ESP_LOGW(TAG, "JPEG 4:2:0 falló, uso 4:2:2");
            s_jenc_422 = true;
        } else if (g_cfg.jpeg_q > 50) {
            g_cfg.jpeg_q -= 5;          // no cupo en el búfer: bajar un poco la calidad
        }
        return;
    }
    if (!publish) return;
    xSemaphoreTake(s_jmux, portMAX_DELAY);
    s_jout_len[k] = len;
    s_jlatest = k;
    s_jseq++;
    xSemaphoreGive(s_jmux);
    if (uvc_format() == 1 && uvc_ready() && uvc_send(s_jout[k], len)) {
        s_jusb = k;
    }
}

size_t pipeline_copy_jpeg(uint8_t *dst, size_t cap, uint32_t *seq_io, TickType_t wait)
{
    s_web_view_t = esp_timer_get_time();
    TickType_t t0 = xTaskGetTickCount();
    while (s_jseq == *seq_io || s_jlatest < 0) {
        if (xTaskGetTickCount() - t0 >= wait) return 0;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    size_t n = 0;
    xSemaphoreTake(s_jmux, portMAX_DELAY);
    int k = s_jlatest;
    n = s_jout_len[k];
    if (n <= cap) memcpy(dst, s_jout[k], n); else n = 0;
    *seq_io = s_jseq;
    xSemaphoreGive(s_jmux);
    return n;
}

// =============================== IA: copia reducida por CPU ===============================
// RGB565 1920x1080 -> BGR888 320x180 (1 de cada 6 píxeles). Se escribe UNA vez y la comparten los 3
// detectores: en este chip copiar con la CPU desde la PSRAM cuesta caro.
// El 2D-DMA recoge 1 de cada 6 filas (lee la PSRAM por filas completas, que es lo eficiente) y las deja
// en RAM interna de a franjas de 12; la CPU submuestrea 1 de cada 6 píxeles desde ahí (rápido).
#define AI_BAND 12
static async_color_convert_handle_t s_cc;
static uint16_t *s_band;                 // AI_BAND filas de 1920 px en RAM interna

static void ai_downscale(const uint16_t *src, uint8_t *dst)
{
    for (int y0 = 0; y0 < AI_H; y0 += AI_BAND) {
        int rows = AI_H - y0 < AI_BAND ? AI_H - y0 : AI_BAND;
        const uint16_t *band = NULL;
        if (s_cc && s_band) {
            async_color_convert_request_t rq = {
                .src_buffer = src + 3 * W, .src_stride = 6 * W, .src_height = AI_H, .src_x = 0, .src_y = y0,
                .dst_buffer = s_band, .dst_stride = W, .dst_height = AI_BAND, .dst_x = 0, .dst_y = 0,
                .copy_width = W, .copy_height = rows,
                .src_color_format = ESP_COLOR_FOURCC_RGB16, .dst_color_format = ESP_COLOR_FOURCC_RGB16,
            };
            int64_t t0 = esp_timer_get_time();
            if (esp_color_convert_blocking(s_cc, &rq, -1) == ESP_OK) band = s_band;
            stats_busy(ENG_DMA2D, t0, esp_timer_get_time());
        }
        for (int r = 0; r < rows; r++) {
            const uint16_t *row = band ? band + (size_t)r * W + 3 : src + (size_t)((y0 + r) * 6 + 3) * W + 3;
            uint8_t *o = dst + (size_t)(y0 + r) * AI_W * 3;
            for (int x = 0; x < AI_W; x++) {
                uint16_t p = row[x * 6];
                uint8_t rr = (p >> 11) & 0x1F, g = (p >> 5) & 0x3F, b = p & 0x1F;
                o[0] = (b << 3) | (b >> 2);
                o[1] = (g << 2) | (g >> 4);
                o[2] = (rr << 3) | (rr >> 2);
                o += 3;
            }
        }
    }
}

static void feed_ai(const uint8_t *frame)
{
    int slot;
    uint8_t *b = ai_pool_begin_write(&slot);
    if (!b) return;
    ai_downscale((const uint16_t *)frame, b);
    ai_pool_publish(slot);
}

// =============================== auto-encuadre ===============================
static void update_framing(const det_list_t *dl, int64_t now)
{
    float tcx = W / 2.0f, tcy = H / 2.0f, tcw = W;
    int best = -1, best_area = 0;
    for (int i = 0; i < dl->n; i++) {
        if (dl->d[i].cls != 0) continue;
        int a = (dl->d[i].x1 - dl->d[i].x0) * (dl->d[i].y1 - dl->d[i].y0);
        if (a > best_area) { best_area = a; best = i; }
    }
    if (g_cfg.autoframe && best >= 0 && now - dl->t_us < 1500000) {
        const det_t *d = &dl->d[best];
        float fx = (d->x0 + d->x1) / 2.0f * W / 10000.0f;
        float fy = (d->y0 + d->y1) / 2.0f * H / 10000.0f;
        float fh = (d->y1 - d->y0) * H / 10000.0f;
        float ch = fh * 3.4f;
        tcw = ch * 16.0f / 9.0f;
        float minw = W / g_cfg.zoom_max;
        if (tcw < minw) tcw = minw;
        if (tcw > W) tcw = W;
        tcx = fx;
        tcy = fy + ch * 0.12f;
        s_last_face_t = now;
    } else if (g_cfg.autoframe && now - s_last_face_t < 2500000) {
        return;
    }
    s_cx += (tcx - s_cx) * 0.10f;
    s_cy += (tcy - s_cy) * 0.10f;
    s_cw += (tcw - s_cw) * 0.06f;
}

// =============================== tarea de salida ===============================
static int64_t s_t_wait, s_t_feed, s_t_zoom, s_t_over, s_t_jpeg, s_t_yuv;
static int s_t_n;
static void out_task(void *arg)
{
    int64_t last_h264 = 0, hud_t = 0;
    while (1) {
        frame_t *f;
        int64_t tw0 = esp_timer_get_time();
        xQueueReceive(s_ready_q, &f, portMAX_DELAY);
        int64_t now = esp_timer_get_time();
        s_t_wait += now - tw0;
        if (f->publish) feed_ai(f->buf);
        int64_t ta = esp_timer_get_time();
        s_t_feed += ta - now;          // la IA trabaja también con el patrón de prueba (mide la CPU)

        det_list_t dl;
        ai_get_dets(&dl);
        if (!f->real) dl.n = 0;
        update_framing(&dl, now);

        // zoom: sólo si hace falta (el PPA cuesta ~20 ms por megapíxel de salida en este chip)
        uint8_t *dst = f->buf;
        size_t dst_cap = s_fb_cap;
        float s = 1.0f;
        int cx = 0, cy = 0, ox = 0, oy = 0;
        if (f->real && s_cw < W * 0.94f) {
            s = floorf((float)W / s_cw * 16.0f) / 16.0f;
            if (s < 1.0625f) s = 1.0625f;
            int cw = ((int)(W / s)) & ~1, ch = ((int)(H / s)) & ~1;
            cx = ((int)(s_cx - cw / 2)) & ~1;
            cy = ((int)(s_cy - ch / 2)) & ~1;
            if (cx < 0) cx = 0;
            if (cy < 0) cy = 0;
            if (cx + cw > W) cx = (W - cw) & ~1;
            if (cy + ch > H) cy = (H - ch) & ~1;
            int bw = (int)(cw * s), bh = (int)(ch * s);
            ox = ((W - bw) / 2) & ~1;
            oy = ((H - bh) / 2) & ~1;
            if (srm(s_srm, f->buf, W, H, cx, cy, cw, ch, PPA_SRM_COLOR_MODE_RGB565, s_comp, s_comp_cap, W, H, ox, oy,
                    PPA_SRM_COLOR_MODE_RGB565, s, g_cfg.mirror)) {
                dst = s_comp;
                dst_cap = s_comp_cap;
            } else {
                s = 1.0f; cx = cy = ox = oy = 0;
            }
        }

        int64_t tb = esp_timer_get_time();
        s_t_zoom += tb - ta;
        if (!f->real) {
            int t = (int)(now / 16000);
            fill_rect(dst, dst_cap, (t * 7) % (W - 160), H / 2 + (int)(sinf(t * 0.05f) * H / 4), 160, 160, 0xFF00D8FF);
        }
        if (g_cfg.boxes) {
            static const uint32_t col[3] = { 0xFF3CFF6E, 0xFFFF9A1F, 0xFF3C9BFF };   // cara, gato, persona
            for (int i = 0; i < dl.n; i++) {
                const det_t *d = &dl.d[i];
                int x0 = (int)(((d->x0 * W / 10000) - cx) * s) + ox, y0 = (int)(((d->y0 * H / 10000) - cy) * s) + oy;
                int x1 = (int)(((d->x1 * W / 10000) - cx) * s) + ox, y1 = (int)(((d->y1 * H / 10000) - cy) * s) + oy;
                if (g_cfg.mirror) { int a = W - x1, b = W - x0; x0 = a; x1 = b; }
                box(dst, dst_cap, x0, y0, x1, y1, col[d->cls % 3], 5);
            }
        }
        if (g_cfg.hud) {
            if (now - hud_t > 500000) { hud_render(); hud_t = now; }
            blend_hud(dst, dst_cap);
        }

        int64_t tc = esp_timer_get_time();
        s_t_over += tc - tb;
        // H.264: cada cuadro en modo máximo o si la webcam lo pide en H.264; si no, 5 por segundo (repetición)
        bool uvc_h264 = uvc_streaming() && uvc_format() == 2;
        int64_t h_interval = (uvc_h264 || g_cfg.max_mode) ? 0 : 200000;
        bool h264_wanted = uvc_h264 || g_cfg.h264_on || g_cfg.max_mode;
        bool yuv = false;
        int64_t t_yuv = 0;
        if (h264_wanted && f->publish && s_h264 && now - last_h264 >= h_interval && xSemaphoreTake(s_yuv_free, 0) == pdTRUE) {
            last_h264 = now;
            s_roi_dets = dl; s_roi_cx = cx; s_roi_cy = cy; s_roi_ox = ox; s_roi_oy = oy; s_roi_s = s;
            t_yuv = esp_timer_get_time();
            yuv = yuv_start(dst);
            if (!yuv) xSemaphoreGive(s_yuv_free);
        }

        // MJPEG (en paralelo con la conversión de arriba): webcam MJPEG, dashboard o vista previa
        bool web = now - s_web_view_t < 3000000;
        if (!uvc_h264 || web || !f->publish) jpeg_publish(dst, f->publish);

        int64_t td = esp_timer_get_time();
        s_t_jpeg += td - tc;
        if (yuv) {
            xSemaphoreTake(s_srm_done, portMAX_DELAY);
            stats_busy(ENG_PPA, t_yuv, esp_timer_get_time());
            xSemaphoreGive(s_yuv_ready);
        }

        int64_t te = esp_timer_get_time();
        s_t_yuv += te - td;
        if (++s_t_n >= 40) {
            ESP_LOGI(TAG, "por cuadro (ms): espera %.1f | IA-copia %.1f | zoom %.1f | cuadros+barra %.1f | JPEG %.1f | espera YUV %.1f",
                     s_t_wait / 1e3 / s_t_n, s_t_feed / 1e3 / s_t_n, s_t_zoom / 1e3 / s_t_n, s_t_over / 1e3 / s_t_n,
                     s_t_jpeg / 1e3 / s_t_n, s_t_yuv / 1e3 / s_t_n);
            s_t_wait = s_t_feed = s_t_zoom = s_t_over = s_t_jpeg = s_t_yuv = 0;
            s_t_n = 0;
        }
        if (f->publish) stats_frame_out(); else stats_filler();
        if (f->publish) {
            int faces = 0, cats = 0;
            for (int i = 0; i < dl.n; i++) { if (dl.d[i].cls == 0) faces++; else if (dl.d[i].cls == 1) cats++; }
            led_state(!f->real ? LED_WAIT : cats ? LED_CAT : faces ? LED_FACE : LED_RUN);
        }
        xQueueSend(s_free_q, &f, portMAX_DELAY);
    }
}

// =============================== tarea de decodificación ===============================
static bool decode_into(frame_t *f, const uint8_t *jpg, size_t len)
{
    jpeg_decode_picture_info_t info;
    if (jpeg_decoder_get_info(jpg, len, &info) != ESP_OK) return false;
    if (info.width != W || info.height > H) {
        static int64_t warned;
        if (esp_timer_get_time() - warned > 5000000) {
            ESP_LOGW(TAG, "el PC mandó %lux%lu; este chip procesa en tiempo real hasta %dx%d",
                     (unsigned long)info.width, (unsigned long)info.height, W, H);
            warned = esp_timer_get_time();
        }
        return false;
    }
    jpeg_decode_cfg_t cfg = {
        .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
        .rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,
        .conv_std = JPEG_YUV_RGB_CONV_STD_BT601,
    };
    uint32_t out = 0;
    int64_t t0 = esp_timer_get_time();
    esp_err_t r = jpeg_decoder_process(s_jdec, &cfg, jpg, len, f->buf, s_fb_cap, &out);
    stats_busy(ENG_JDEC, t0, esp_timer_get_time());
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "JPEG no decodificable: %s", esp_err_to_name(r));
        return false;
    }
    f->w = info.width;
    f->h = info.height;
    return true;
}

static void make_bench_jpeg(void)
{
    // patrón de prueba 1920x1080 (franjas + cuadrícula), codificado una vez con el JPEG HW
    uint8_t *b = s_fb[0].buf;
    uint32_t cols[8] = { 0xFFE8E8E8, 0xFFE8E800, 0xFF00E8E8, 0xFF00E800, 0xFFE800E8, 0xFFE80000, 0xFF0000E8, 0xFF101010 };
    for (int i = 0; i < 8; i++) fill_rect(b, s_fb_cap, i * (W / 8), 0, W / 8, H, cols[i]);
    for (int gx = 0; gx < W; gx += 120) fill_rect(b, s_fb_cap, gx, 0, 3, H, 0xFF202020);
    for (int gy = 0; gy < H; gy += 120) fill_rect(b, s_fb_cap, 0, gy, W, 3, 0xFF202020);
    jpeg_encode_cfg_t cfg = { .height = H, .width = W, .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
                              .sub_sample = JPEG_DOWN_SAMPLING_YUV420, .image_quality = 85 };
    uint32_t len = 0;
    if (jpeg_encoder_process(s_jenc, &cfg, b, W * H * 2, s_jout[0], s_jout_cap, &len) == ESP_OK) {
        jpeg_decode_memory_alloc_cfg_t dc = { .buffer_direction = JPEG_DEC_ALLOC_INPUT_BUFFER };
        size_t got = 0;
        s_bench_jpeg = jpeg_alloc_decoder_mem(len, &dc, &got);
        memcpy(s_bench_jpeg, s_jout[0], len);
        s_bench_len = len;
        ESP_LOGI(TAG, "patrón de prueba %dx%d listo (%lu KB)", W, H, (unsigned long)len / 1024);
    }
}

static void dec_task(void *arg)
{
    make_bench_jpeg();
    int64_t last_real = 0, last_bench = 0;
    while (1) {
        jpeg_in_t *in = ingest_get(pdMS_TO_TICKS(g_cfg.max_mode ? 0 : 15));
        int64_t now = esp_timer_get_time();
        bool pc_idle = now - last_real > 1000000;
        const uint8_t *jpg = NULL;
        size_t len = 0;
        bool real = false;
        if (in) {
            jpg = in->buf; len = in->len; real = true;
        } else if (s_bench_jpeg && (g_cfg.max_mode || (pc_idle && now - last_bench > 33000))) {
            jpg = s_bench_jpeg; len = s_bench_len;
            last_bench = now;
        } else {
            continue;
        }
        frame_t *f;
        int64_t tq = esp_timer_get_time();
        xQueueReceive(s_free_q, &f, portMAX_DELAY);
        int64_t tq2 = esp_timer_get_time();
        bool ok = decode_into(f, jpg, len);
        static int64_t s_dq, s_dd; static int s_dn;
        s_dq += tq2 - tq; s_dd += esp_timer_get_time() - tq2;
        if (++s_dn >= 40) {
            ESP_LOGI(TAG, "decodificador (ms): espera búfer %.1f | decodificar %.1f", s_dq / 1e3 / s_dn, s_dd / 1e3 / s_dn);
            s_dq = s_dd = 0; s_dn = 0;
        }
        if (in) ingest_release(in);
        if (!ok) {
            xQueueSend(s_free_q, &f, portMAX_DELAY);
            continue;
        }
        if (real) last_real = now;
        f->real = real;
        f->publish = real || pc_idle;
        f->t_rx = now;
        xQueueSend(s_ready_q, &f, portMAX_DELAY);
    }
}

// =============================== arranque ===============================
void pipeline_start(void)
{
    jpeg_decode_engine_cfg_t dcfg = { .timeout_ms = 200 };
    ESP_ERROR_CHECK(jpeg_new_decoder_engine(&dcfg, &s_jdec));
    jpeg_encode_engine_cfg_t ecfg = { .timeout_ms = 200 };
    ESP_ERROR_CHECK(jpeg_new_encoder_engine(&ecfg, &s_jenc));
    ppa_client_config_t pc = { .oper_type = PPA_OPERATION_SRM, .max_pending_trans_num = 1 };
    ESP_ERROR_CHECK(ppa_register_client(&pc, &s_srm));
    ESP_ERROR_CHECK(ppa_register_client(&pc, &s_srm_yuv));
    pc.oper_type = PPA_OPERATION_FILL;
    ESP_ERROR_CHECK(ppa_register_client(&pc, &s_fill));
    pc.oper_type = PPA_OPERATION_BLEND;
    ESP_ERROR_CHECK(ppa_register_client(&pc, &s_blend));

    jpeg_decode_memory_alloc_cfg_t dm = { .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER };
    s_free_q = xQueueCreate(N_FB, sizeof(frame_t *));
    s_ready_q = xQueueCreate(N_FB, sizeof(frame_t *));
    for (int i = 0; i < N_FB; i++) {
        s_fb[i].buf = jpeg_alloc_decoder_mem(W * H16 * 2, &dm, &s_fb_cap);
        frame_t *p = &s_fb[i];
        xQueueSend(s_free_q, &p, 0);
    }
    s_comp_cap = P4_ALIGN(W * H16 * 2, 64);
    s_comp = alloc64(s_comp_cap);
    s_yuv_cap = P4_ALIGN(W * H16 * 3 / 2, 64);
    s_yuv = alloc64(s_yuv_cap);
    jpeg_encode_memory_alloc_cfg_t em = { .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER };
    for (int i = 0; i < N_JOUT; i++) s_jout[i] = jpeg_alloc_encoder_mem(640 * 1024, &em, &s_jout_cap);
    s_hout_cap = 640 * 1024;
    uint32_t act = 0;
    s_hout = esp_h264_aligned_calloc(64, 1, s_hout_cap, &act, ESP_H264_MEM_SPIRAM);
    s_ring = heap_caps_malloc(RING_CAP, MALLOC_CAP_SPIRAM);
    s_rf = heap_caps_calloc(RF_MAX, sizeof(rframe_t), MALLOC_CAP_SPIRAM);
    s_jmux = xSemaphoreCreateMutex();
    s_ring_mux = xSemaphoreCreateMutex();
    s_yuv_ready = xSemaphoreCreateBinary();
    s_yuv_free = xSemaphoreCreateBinary();
    xSemaphoreGive(s_yuv_free);
    s_srm_done = xSemaphoreCreateBinary();
    ppa_event_callbacks_t cbs = { .on_trans_done = on_yuv_done };
    ppa_client_register_event_callbacks(s_srm_yuv, &cbs);
    if (!s_fb[N_FB - 1].buf || !s_comp || !s_yuv || !s_jout[N_JOUT - 1] || !s_hout || !s_ring || !s_rf) {
        ESP_LOGE(TAG, "falta memoria para la tubería (PSRAM libre %u KB)",
                 (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
        abort();
    }
    async_color_convert_config_t ccc = { .backlog = 2, .dma_burst_size = 128 };
    if (esp_async_color_convert_install_dma2d(&ccc, &s_cc) != ESP_OK) s_cc = NULL;
    s_band = heap_caps_aligned_calloc(64, 1, AI_BAND * W * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    hud_init(W);
    ESP_LOGI(TAG, "tubería lista (%dx%d, 3 tareas en paralelo); PSRAM libre %u KB", W, H,
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
    // prioridades: USB 20 > red 18 > web 12 > entrada 11 > dec 10 > out 9 (núcleo 0); H.264 12 (núcleo 1)
    xTaskCreatePinnedToCore(dec_task, "dec", 6144, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(out_task, "out", 8192, NULL, 9, NULL, 0);
    xTaskCreatePinnedToCore(h264_task, "h264", 6144, NULL, 12, NULL, 1);
}
