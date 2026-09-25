// Tubería de video del Cerebro P4. Por cada cuadro:
//   JPEG (del PC) --[decodificador JPEG HW]--> RGB565 hasta 2560x1440
//     --[PPA escala]--> 640x360 BGR888 para las IA (caras y gatos, en la CPU con instrucciones PIE)
//     --[PPA recorte + zoom + espejo]--> salida 1920x1080 que sigue tu cara
//     --[PPA relleno]--> cuadros de detección   --[PPA mezcla alfa]--> barra de información
//     --[codificador JPEG HW]--> MJPEG para la webcam y el dashboard
//     --[PPA RGB->YUV420]--[codificador H.264 HW, con ROI en caras y vectores de movimiento]
//          --> webcam H.264 + grabación continua de los últimos segundos (repetición)
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
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

#define SRC_MAX_W 2560
#define SRC_MAX_H 1440
#define OUT_MAX_W 1920
#define OUT_MAX_H 1088
#define BENCH_W   2560
#define BENCH_H   1440

// ---------- motores ----------
static jpeg_decoder_handle_t s_jdec;
static jpeg_encoder_handle_t s_jenc;
static ppa_client_handle_t   s_srm, s_fill, s_blend;
static esp_h264_enc_handle_t s_h264;
static esp_h264_enc_param_hw_handle_t s_h264p;
static int s_h264_w, s_h264_h, s_h264_kbps;
static bool s_jenc_422;              // si el 4:2:0 falla con 1080 líneas, se usa 4:2:2

// ---------- memoria ----------
static uint8_t *s_dec;  static size_t s_dec_cap;
static uint8_t *s_comp; static size_t s_comp_cap;
static uint8_t *s_yuv;  static size_t s_yuv_cap;
static uint8_t *s_bench_jpeg; static size_t s_bench_len;

#define N_JOUT 3
static uint8_t *s_jout[N_JOUT];
static size_t   s_jout_cap, s_jout_len[N_JOUT];
static int      s_jlatest = -1, s_jusb = -1;
static volatile uint32_t s_jseq;
static SemaphoreHandle_t s_jmux;

#define N_HOUT 2
static uint8_t *s_hout;  static size_t s_hout_cap;
static uint8_t *s_hq[N_HOUT]; static int s_hq_i;
static bool     s_uvc_h264_synced;

static esp_h264_enc_mv_data_t *s_mv; static uint32_t s_mv_cap;
static volatile float s_motion_pct;

// ---------- repetición: anillo de cuadros H.264 ----------
#define RING_CAP (4 * 1024 * 1024)      // ~8 s de video a 4 Mbps
#define RF_MAX   2048
typedef struct { uint32_t off, len, id; int64_t t; uint8_t idr; } rframe_t;
static uint8_t *s_ring;
static uint32_t s_ring_head;
static rframe_t *s_rf;                 // índice de cuadros (en PSRAM)
static int      s_rf_head, s_rf_count;
static uint32_t s_rf_id;
static SemaphoreHandle_t s_ring_mux;

// ---------- auto-encuadre ----------
static float   s_cx, s_cy, s_cw;     // centro y ancho del recorte (px de la fuente), suavizados
static int64_t s_last_face_t;
static int     s_src_w, s_src_h;

static void *alloc64(size_t n)
{
    return heap_caps_aligned_calloc(64, 1, P4_ALIGN(n, 64), MALLOC_CAP_SPIRAM);
}

// ===================================================================================
static bool srm(const void *in, int in_w, int in_h, int bx, int by, int bw, int bh, ppa_srm_color_mode_t in_cm,
                void *out, size_t out_cap, int out_w, int out_h, int ox, int oy, ppa_srm_color_mode_t out_cm,
                float scale, bool mirror)
{
    ppa_srm_oper_config_t op = {
        .in = { .buffer = in, .pic_w = in_w, .pic_h = in_h, .block_w = bw, .block_h = bh,
                .block_offset_x = bx, .block_offset_y = by, .srm_cm = in_cm },
        .out = { .buffer = out, .buffer_size = out_cap, .pic_w = out_w, .pic_h = out_h,
                 .block_offset_x = ox, .block_offset_y = oy, .srm_cm = out_cm },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
        .scale_x = scale, .scale_y = scale,
        .mirror_x = mirror, .mirror_y = false,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };
    int64_t t0 = esp_timer_get_time();
    esp_err_t r = ppa_do_scale_rotate_mirror(s_srm, &op);
    stats_busy(ENG_PPA, t0, esp_timer_get_time());
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "PPA SRM falló (%s) in %dx%d blk %d,%d %dx%d -> out %dx%d esc %.4f",
                 esp_err_to_name(r), in_w, in_h, bx, by, bw, bh, out_w, out_h, scale);
    }
    return r == ESP_OK;
}

static void fill_rect(int out_w, int out_h, int x, int y, int w, int h, uint32_t argb)
{
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > out_w) w = out_w - x;
    if (y + h > out_h) h = out_h - y;
    if (w <= 0 || h <= 0) return;
    ppa_fill_oper_config_t op = {
        .out = { .buffer = s_comp, .buffer_size = s_comp_cap, .pic_w = out_w, .pic_h = out_h,
                 .block_offset_x = x, .block_offset_y = y, .fill_cm = PPA_FILL_COLOR_MODE_RGB565 },
        .fill_block_w = w, .fill_block_h = h,
        .fill_argb_color = { .val = argb },
        .mode = PPA_TRANS_MODE_BLOCKING,
    };
    int64_t t0 = esp_timer_get_time();
    ppa_do_fill(s_fill, &op);
    stats_busy(ENG_PPA, t0, esp_timer_get_time());
}

static void box(int out_w, int out_h, int x0, int y0, int x1, int y1, uint32_t argb, int t)
{
    fill_rect(out_w, out_h, x0, y0, x1 - x0, t, argb);
    fill_rect(out_w, out_h, x0, y1 - t, x1 - x0, t, argb);
    fill_rect(out_w, out_h, x0, y0, t, y1 - y0, argb);
    fill_rect(out_w, out_h, x1 - t, y0, t, y1 - y0, argb);
}

static void blend_hud(int out_w, int out_h)
{
    int hw, hh;
    uint8_t *hud = hud_buf(&hw, &hh);
    if (!hud || hw != out_w) return;
    ppa_blend_oper_config_t op = {
        .in_bg = { .buffer = s_comp, .pic_w = out_w, .pic_h = out_h, .block_w = out_w, .block_h = hh,
                   .block_offset_x = 0, .block_offset_y = 0, .blend_cm = PPA_BLEND_COLOR_MODE_RGB565 },
        .in_fg = { .buffer = hud, .pic_w = hw, .pic_h = hh, .block_w = hw, .block_h = hh,
                   .block_offset_x = 0, .block_offset_y = 0, .blend_cm = PPA_BLEND_COLOR_MODE_ARGB8888 },
        .out = { .buffer = s_comp, .buffer_size = s_comp_cap, .pic_w = out_w, .pic_h = out_h,
                 .block_offset_x = 0, .block_offset_y = 0, .blend_cm = PPA_BLEND_COLOR_MODE_RGB565 },
        .bg_alpha_update_mode = PPA_ALPHA_NO_CHANGE,
        .fg_alpha_update_mode = PPA_ALPHA_NO_CHANGE,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };
    int64_t t0 = esp_timer_get_time();
    ppa_do_blend(s_blend, &op);
    stats_busy(ENG_PPA, t0, esp_timer_get_time());
}

// ===================================================================================
static void ring_append(const uint8_t *p, uint32_t len, bool idr)
{
    if (len == 0 || len > RING_CAP / 4) return;
    xSemaphoreTake(s_ring_mux, portMAX_DELAY);
    uint32_t off = s_ring_head;
    if (off + len > RING_CAP) off = 0;
    // descartar los cuadros viejos que se van a pisar
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
    xSemaphoreGive(s_ring_mux);
}

int replay_stream(int (*emit)(void *ctx, const uint8_t *p, size_t n), void *ctx, int seconds)
{
    uint8_t *tmp = heap_caps_malloc(RING_CAP / 4, MALLOC_CAP_SPIRAM);
    if (!tmp) return -1;
    int64_t since = esp_timer_get_time() - (int64_t)seconds * 1000000;
    xSemaphoreTake(s_ring_mux, portMAX_DELAY);
    // empezar en el último IDR anterior a "since" (así el clip dura al menos lo pedido)
    uint32_t start = UINT32_MAX, first_idr = UINT32_MAX, last = s_rf_id;
    for (int i = s_rf_count; i > 0; i--) {                  // del más viejo al más nuevo
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

// ===================================================================================
static bool h264_setup(int w, int h)
{
    if (s_h264) {
        if (s_h264_kbps != g_cfg.h264_kbps) {
            esp_h264_enc_set_bitrate((esp_h264_enc_param_handle_t)s_h264p, g_cfg.h264_kbps * 1000);
            s_h264_kbps = g_cfg.h264_kbps;
        }
        return s_h264_w == w && s_h264_h == h;
    }
    esp_h264_enc_cfg_hw_t cfg = {
        .pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY,
        .gop = 30,
        .fps = 30,
        .res = { .width = w, .height = h },
        .rc = { .bitrate = g_cfg.h264_kbps * 1000, .qp_min = 18, .qp_max = 38 },
    };
    if (esp_h264_enc_hw_new(&cfg, &s_h264) != ESP_H264_ERR_OK || esp_h264_enc_open(s_h264) != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "no se pudo abrir el H.264 %dx%d", w, h);
        s_h264 = NULL;
        return false;
    }
    esp_h264_enc_hw_get_param_hd(s_h264, &s_h264p);
    // calidad extra donde la IA ve caras (ROI) y vectores de movimiento como sensor de movimiento
    esp_h264_enc_roi_cfg_t roi = { .roi_mode = ESP_H264_ROI_MODE_DELTA_QP, .none_roi_delta_qp = 3 };
    esp_h264_enc_hw_cfg_roi(s_h264p, roi);
    esp_h264_enc_mv_cfg_t mv = { .mv_mode = ESP_H264_MVM_MODE_P16X16, .mv_fmt = ESP_H264_MVM_FMT_PART };
    esp_h264_enc_hw_cfg_mv(s_h264p, mv);
    s_mv_cap = (w / 16) * (P4_ALIGN(h, 16) / 16) * sizeof(esp_h264_enc_mv_data_t);
    free(s_mv);
    s_mv = heap_caps_aligned_calloc(64, 1, P4_ALIGN(s_mv_cap, 64), MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    s_h264_w = w; s_h264_h = h; s_h264_kbps = g_cfg.h264_kbps;
    s_uvc_h264_synced = false;
    ESP_LOGI(TAG, "H.264 %dx%d a %d kbps (ROI en caras + vectores de movimiento)", w, h, g_cfg.h264_kbps);
    return true;
}

static void h264_set_roi(const det_list_t *dl, int crop_x, int crop_y, float s, int ox, int oy, int out_w, int out_h)
{
    int mbw = out_w / 16, mbh = P4_ALIGN(out_h, 16) / 16;
    int k = 0;
    for (int i = 0; i < dl->n && k < 8; i++) {
        if (dl->d[i].cls != 0) continue;
        int x0 = (int)(((dl->d[i].x0 * s_src_w / 10000) - crop_x) * s) + ox;
        int y0 = (int)(((dl->d[i].y0 * s_src_h / 10000) - crop_y) * s) + oy;
        int x1 = (int)(((dl->d[i].x1 * s_src_w / 10000) - crop_x) * s) + ox;
        int y1 = (int)(((dl->d[i].y1 * s_src_h / 10000) - crop_y) * s) + oy;
        if (g_cfg.mirror) { int a = out_w - x1, b = out_w - x0; x0 = a; x1 = b; }
        int mx = x0 / 16, my = y0 / 16, lx = (x1 - x0) / 16 + 1, ly = (y1 - y0) / 16 + 1;
        if (mx < 0) { lx += mx; mx = 0; }
        if (my < 0) { ly += my; my = 0; }
        if (mx + lx > mbw) lx = mbw - mx;
        if (my + ly > mbh) ly = mbh - my;
        if (lx <= 0 || ly <= 0) continue;
        esp_h264_enc_roi_reg_t r = { .x = mx, .y = my, .len_x = lx, .len_y = ly, .qp = -6, .reg_idx = k++ };
        esp_h264_enc_hw_set_roi_region(s_h264p, r);
    }
    for (; k < 8; k++) {                  // regiones sin usar: tamaño cero
        esp_h264_enc_roi_reg_t r = { .x = 0, .y = 0, .len_x = 0, .len_y = 0, .qp = 0, .reg_idx = k };
        esp_h264_enc_hw_set_roi_region(s_h264p, r);
    }
}

static void h264_encode(int out_w, int out_h, bool publish)
{
    if (!h264_setup(out_w, out_h)) return;
    // RGB565 -> YUV420 (formato O_UYY_E_VYY) con el PPA
    if (!srm(s_comp, out_w, out_h, 0, 0, out_w, out_h, PPA_SRM_COLOR_MODE_RGB565,
             s_yuv, s_yuv_cap, out_w, out_h, 0, 0, PPA_SRM_COLOR_MODE_YUV420, 1.0f, false)) return;
    esp_h264_enc_in_frame_t in = { .raw_data = { .buffer = s_yuv, .len = out_w * P4_ALIGN(out_h, 16) * 3 / 2 },
                                   .pts = (uint32_t)(esp_timer_get_time() / 1000) };
    esp_h264_enc_out_frame_t out = { .raw_data = { .buffer = s_hout, .len = s_hout_cap } };
    esp_h264_enc_mvm_pkt_t pkt = { .data = s_mv, .len = s_mv_cap };
    esp_h264_enc_hw_set_mv_pkt(s_h264p, pkt);
    int64_t t0 = esp_timer_get_time();
    esp_h264_err_t r = esp_h264_enc_process(s_h264, &in, &out);
    stats_busy(ENG_H264, t0, esp_timer_get_time());
    if (r != ESP_H264_ERR_OK) {
        ESP_LOGW(TAG, "H.264 falló (%d)", r);
        return;
    }
    uint32_t mvlen = 0;
    esp_h264_enc_hw_get_mv_data_len(s_h264p, &mvlen);
    int total_mb = (out_w / 16) * (P4_ALIGN(out_h, 16) / 16);
    float m = 100.0f * (float)(mvlen / sizeof(esp_h264_enc_mv_data_t)) / (float)total_mb;
    s_motion_pct = s_motion_pct * 0.8f + m * 0.2f;

    bool idr = out.frame_type == ESP_H264_FRAME_TYPE_IDR;
    if (!publish) return;
    ring_append(out.raw_data.buffer, out.length, idr);
    if (uvc_format() == 2 && uvc_streaming()) {
        if (idr) s_uvc_h264_synced = true;
        if (s_uvc_h264_synced && uvc_ready() && out.length <= s_hout_cap) {
            s_hq_i = (s_hq_i + 1) % N_HOUT;
            memcpy(s_hq[s_hq_i], out.raw_data.buffer, out.length);
            uvc_send(s_hq[s_hq_i], out.length);
        } else if (!uvc_ready()) {
            // el PC va atrasado: se salta hasta el próximo IDR para no corromper la imagen
            s_uvc_h264_synced = false;
        }
    } else {
        s_uvc_h264_synced = false;
    }
}

static int jpeg_encode(int out_w, int out_h, bool publish)
{
    // elegir un búfer que no esté viajando por USB
    int k = 0;
    for (int i = 1; i <= N_JOUT; i++) {
        int c = (s_jlatest + i) % N_JOUT;
        if (c < 0) c += N_JOUT;
        if (c != s_jusb || uvc_ready()) { k = c; break; }
    }
    jpeg_encode_cfg_t cfg = {
        .height = out_h, .width = out_w,
        .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
        .sub_sample = s_jenc_422 ? JPEG_DOWN_SAMPLING_YUV422 : JPEG_DOWN_SAMPLING_YUV420,
        .image_quality = g_cfg.jpeg_q,
    };
    uint32_t len = 0;
    int64_t t0 = esp_timer_get_time();
    esp_err_t r = jpeg_encoder_process(s_jenc, &cfg, s_comp, out_w * out_h * 2, s_jout[k], s_jout_cap, &len);
    stats_busy(ENG_JENC, t0, esp_timer_get_time());
    if (r != ESP_OK) {
        if (!s_jenc_422) {
            ESP_LOGW(TAG, "JPEG 4:2:0 falló con %dx%d, uso 4:2:2", out_w, out_h);
            s_jenc_422 = true;
        }
        return -1;
    }
    if (!publish) return k;
    xSemaphoreTake(s_jmux, portMAX_DELAY);
    s_jout_len[k] = len;
    s_jlatest = k;
    s_jseq++;
    xSemaphoreGive(s_jmux);
    if (uvc_format() == 1 && uvc_ready()) {
        if (uvc_send(s_jout[k], len)) s_jusb = k;
    }
    return k;
}

size_t pipeline_copy_jpeg(uint8_t *dst, size_t cap, uint32_t *seq_io, TickType_t wait)
{
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

void pipeline_snapshot_request(void) {}

// ===================================================================================
static bool decode(const uint8_t *jpg, size_t len, int *w, int *h)
{
    jpeg_decode_picture_info_t info;
    if (jpeg_decoder_get_info(jpg, len, &info) != ESP_OK) return false;
    if (info.width > SRC_MAX_W || info.height > SRC_MAX_H) {
        ESP_LOGW(TAG, "cuadro %lux%lu excede el máximo", (unsigned long)info.width, (unsigned long)info.height);
        return false;
    }
    jpeg_decode_cfg_t cfg = {
        .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
        .rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,
        .conv_std = JPEG_YUV_RGB_CONV_STD_BT601,
    };
    uint32_t out = 0;
    int64_t t0 = esp_timer_get_time();
    esp_err_t r = jpeg_decoder_process(s_jdec, &cfg, jpg, len, s_dec, s_dec_cap, &out);
    stats_busy(ENG_JDEC, t0, esp_timer_get_time());
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "decodificación JPEG falló: %s", esp_err_to_name(r));
        return false;
    }
    *w = info.width;
    *h = info.height;
    return true;
}

static void feed_ai(int w, int h)
{
    for (int which = 0; which < 3; which++) {
        if (!ai_wants_frame(which)) continue;
        float s = floorf((float)AI_W / w * 16.0f) / 16.0f;
        if (s * h > AI_H) s = floorf((float)AI_H / h * 16.0f) / 16.0f;
        int aw = (int)(w * s) & ~1, ah = (int)(h * s) & ~1;
        if (srm(s_dec, w, h, 0, 0, w, h, PPA_SRM_COLOR_MODE_RGB565,
                ai_buffer(which), P4_ALIGN(AI_W * AI_H * 3, 64), aw, ah, 0, 0, PPA_SRM_COLOR_MODE_RGB888, s, false)) {
            ai_frame_ready(which, aw, ah);
        }
    }
}

static void update_framing(const det_list_t *dl, int w, int h, int64_t now)
{
    float tcx = w / 2.0f, tcy = h / 2.0f, tcw = w;
    int best = -1, best_area = 0;
    for (int i = 0; i < dl->n; i++) {
        if (dl->d[i].cls != 0) continue;
        int a = (dl->d[i].x1 - dl->d[i].x0) * (dl->d[i].y1 - dl->d[i].y0);
        if (a > best_area) { best_area = a; best = i; }
    }
    if (g_cfg.autoframe && best >= 0 && now - dl->t_us < 1500000) {
        const det_t *d = &dl->d[best];
        float fx = (d->x0 + d->x1) / 2.0f * w / 10000.0f;
        float fy = (d->y0 + d->y1) / 2.0f * h / 10000.0f;
        float fh = (d->y1 - d->y0) * h / 10000.0f;
        float ch = fh * 3.4f;                    // la cara ocupa ~30% del alto
        tcw = ch * 16.0f / 9.0f;
        float minw = w / g_cfg.zoom_max;
        if (tcw < minw) tcw = minw;
        if (tcw > w) tcw = w;
        tcx = fx;
        tcy = fy + ch * 0.12f;                   // aire arriba de la cabeza
        s_last_face_t = now;
    } else if (g_cfg.autoframe && now - s_last_face_t < 2500000) {
        return;                                  // se perdió la cara hace poco: quedarse quieto
    }
    if (s_cw <= 0 || w != s_src_w || h != s_src_h) { s_cx = tcx; s_cy = tcy; s_cw = tcw; }
    s_cx += (tcx - s_cx) * 0.10f;
    s_cy += (tcy - s_cy) * 0.10f;
    s_cw += (tcw - s_cw) * 0.06f;
}

static void process_frame(int w, int h, bool real, bool publish, int out_w, int out_h)
{
    int64_t now = esp_timer_get_time();
    if (real) feed_ai(w, h);

    det_list_t dl;
    ai_get_dets(&dl);
    if (!real) dl.n = 0;
    update_framing(&dl, w, h, now);
    s_src_w = w; s_src_h = h;

    // escala cuantizada a 1/16 (resolución del PPA) y recorte que llena la salida
    float want = (float)out_w / (s_cw > 0 ? s_cw : w);
    float s = floorf(want * 16.0f) / 16.0f;
    float smin = floorf((float)out_w / w * 16.0f) / 16.0f;
    if (s < smin) s = smin;
    if (s < 0.0625f) s = 0.0625f;
    int cw = (int)(out_w / s), ch = (int)(out_h / s);
    if (cw > w) cw = w;
    if (ch > h) ch = h;
    cw &= ~1; ch &= ~1;
    int cx = (int)(s_cx - cw / 2), cy = (int)(s_cy - ch / 2);
    if (cx < 0) cx = 0;
    if (cy < 0) cy = 0;
    if (cx + cw > w) cx = w - cw;
    if (cy + ch > h) cy = h - ch;
    cx &= ~1; cy &= ~1;
    int bw = (int)(cw * s), bh = (int)(ch * s);
    int ox = ((out_w - bw) / 2) & ~1, oy = ((out_h - bh) / 2) & ~1;
    if (bw < out_w || bh < out_h) fill_rect(out_w, out_h, 0, 0, out_w, out_h, 0xFF000000);
    srm(s_dec, w, h, cx, cy, cw, ch, PPA_SRM_COLOR_MODE_RGB565,
        s_comp, s_comp_cap, out_w, out_h, ox, oy, PPA_SRM_COLOR_MODE_RGB565, s, g_cfg.mirror);

    if (!real) {
        // cuadro de prueba: un bloque que se mueve para que se note que está vivo
        int t = (int)(now / 16000);
        int bx = (t * 7) % (out_w - 160), by = out_h / 2 + (int)(sinf(t * 0.05f) * out_h / 4);
        fill_rect(out_w, out_h, bx, by, 160, 160, 0xFF00D8FF);
    }

    if (g_cfg.boxes) {
        for (int i = 0; i < dl.n; i++) {
            const det_t *d = &dl.d[i];
            int x0 = (int)(((d->x0 * w / 10000) - cx) * s) + ox;
            int y0 = (int)(((d->y0 * h / 10000) - cy) * s) + oy;
            int x1 = (int)(((d->x1 * w / 10000) - cx) * s) + ox;
            int y1 = (int)(((d->y1 * h / 10000) - cy) * s) + oy;
            if (g_cfg.mirror) { int a = out_w - x1, b = out_w - x0; x0 = a; x1 = b; }
            static const uint32_t col[3] = { 0xFF3CFF6E, 0xFFFF9A1F, 0xFF3C9BFF };   // cara verde, gato naranjo, persona azul
            box(out_w, out_h, x0, y0, x1, y1, col[d->cls % 3], out_w >= 1920 ? 5 : 4);
        }
    }

    static int64_t s_hud_t;
    if (g_cfg.hud) {
        if (now - s_hud_t > 250000) { hud_render(); s_hud_t = now; }
        blend_hud(out_w, out_h);
    }

    jpeg_encode(out_w, out_h, publish);
    if (publish) {
        // los cuadros de relleno NO pasan por el H.264: ensuciarían las referencias del video real
        if (s_h264p && real) h264_set_roi(&dl, cx, cy, s, ox, oy, out_w, out_h);
        h264_encode(out_w, out_h, true);
        stats_frame_out();
    } else {
        stats_filler();
    }

    int faces = 0, cats = 0;
    for (int i = 0; i < dl.n; i++) { if (dl.d[i].cls == 0) faces++; else if (dl.d[i].cls == 1) cats++; }
    if (publish) led_state(!real ? LED_WAIT : cats ? LED_CAT : faces ? LED_FACE : LED_RUN);
}

// ===================================================================================
static void make_bench_jpeg(void)
{
    // patrón de prueba 2560x1440: franjas de color, cuadrícula y un marco; se codifica una vez
    const int w = BENCH_W, h = BENCH_H;
    uint32_t cols[8] = { 0xFFE8E8E8, 0xFFE8E800, 0xFF00E8E8, 0xFF00E800, 0xFFE800E8, 0xFFE80000, 0xFF0000E8, 0xFF101010 };
    for (int i = 0; i < 8; i++) {
        ppa_fill_oper_config_t op = {
            .out = { .buffer = s_dec, .buffer_size = s_dec_cap, .pic_w = w, .pic_h = h,
                     .block_offset_x = i * (w / 8), .block_offset_y = 0, .fill_cm = PPA_FILL_COLOR_MODE_RGB565 },
            .fill_block_w = w / 8, .fill_block_h = h, .fill_argb_color = { .val = cols[i] },
            .mode = PPA_TRANS_MODE_BLOCKING,
        };
        ppa_do_fill(s_fill, &op);
    }
    for (int gx = 0; gx < w; gx += 160) {
        ppa_fill_oper_config_t op = {
            .out = { .buffer = s_dec, .buffer_size = s_dec_cap, .pic_w = w, .pic_h = h,
                     .block_offset_x = gx, .block_offset_y = 0, .fill_cm = PPA_FILL_COLOR_MODE_RGB565 },
            .fill_block_w = 4, .fill_block_h = h, .fill_argb_color = { .val = 0xFF202020 },
            .mode = PPA_TRANS_MODE_BLOCKING,
        };
        ppa_do_fill(s_fill, &op);
    }
    for (int gy = 0; gy < h; gy += 160) {
        ppa_fill_oper_config_t op = {
            .out = { .buffer = s_dec, .buffer_size = s_dec_cap, .pic_w = w, .pic_h = h,
                     .block_offset_x = 0, .block_offset_y = gy, .fill_cm = PPA_FILL_COLOR_MODE_RGB565 },
            .fill_block_w = w, .fill_block_h = 4, .fill_argb_color = { .val = 0xFF202020 },
            .mode = PPA_TRANS_MODE_BLOCKING,
        };
        ppa_do_fill(s_fill, &op);
    }
    jpeg_encode_memory_alloc_cfg_t mc = { .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER };
    size_t cap = 0;
    uint8_t *tmp = jpeg_alloc_encoder_mem(1024 * 1024, &mc, &cap);
    jpeg_encode_cfg_t cfg = { .height = h, .width = w, .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
                              .sub_sample = JPEG_DOWN_SAMPLING_YUV420, .image_quality = 85 };
    uint32_t len = 0;
    if (jpeg_encoder_process(s_jenc, &cfg, s_dec, w * h * 2, tmp, cap, &len) == ESP_OK) {
        jpeg_decode_memory_alloc_cfg_t dc = { .buffer_direction = JPEG_DEC_ALLOC_INPUT_BUFFER };
        size_t got = 0;
        s_bench_jpeg = jpeg_alloc_decoder_mem(len, &dc, &got);
        memcpy(s_bench_jpeg, tmp, len);
        s_bench_len = len;
        ESP_LOGI(TAG, "patrón de prueba %dx%d listo (%lu KB)", w, h, (unsigned long)len / 1024);
    } else {
        ESP_LOGE(TAG, "no se pudo generar el patrón de prueba");
    }
    free(tmp);
}

static void bench_engines(void)
{
    // cuánto tarda cada motor por separado (se imprime una vez al arrancar)
    int w, h;
    int64_t t0 = esp_timer_get_time();
    bool ok = decode(s_bench_jpeg, s_bench_len, &w, &h);
    int64_t t1 = esp_timer_get_time();
    srm(s_dec, w, h, 0, 0, w, h, PPA_SRM_COLOR_MODE_RGB565, s_comp, s_comp_cap, 1920, 1080, 0, 0,
        PPA_SRM_COLOR_MODE_RGB565, 0.75f, false);
    int64_t t2 = esp_timer_get_time();
    srm(s_comp, 1920, 1080, 0, 0, 1920, 1080, PPA_SRM_COLOR_MODE_RGB565, s_yuv, s_yuv_cap, 1920, 1080, 0, 0,
        PPA_SRM_COLOR_MODE_YUV420, 1.0f, false);
    int64_t t3 = esp_timer_get_time();
    hud_render();
    int64_t t4 = esp_timer_get_time();
    blend_hud(1920, 1080);
    int64_t t5 = esp_timer_get_time();
    jpeg_encode(1920, 1080, false);
    int64_t t6 = esp_timer_get_time();
    esp_cache_msync(s_dec, s_dec_cap, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    int64_t t7 = esp_timer_get_time();
    float s = 0.25f;
    srm(s_dec, w, h, 0, 0, w, h, PPA_SRM_COLOR_MODE_RGB565, ai_buffer(0), P4_ALIGN(AI_W * AI_H * 3, 64), 640, 360, 0, 0,
        PPA_SRM_COLOR_MODE_RGB888, s, false);
    int64_t t8 = esp_timer_get_time();
    ESP_LOGI(TAG, "BENCH ok=%d %dx%d | JPEGdec %.1f ms | PPA 1440p->1080p %.1f ms | PPA RGB->YUV %.1f ms | "
             "HUD cpu %.1f ms | PPA mezcla %.1f ms | JPEGenc 1080p %.1f ms | msync 7MB %.1f ms | PPA ->IA %.1f ms",
             ok, w, h, (t1 - t0) / 1e3, (t2 - t1) / 1e3, (t3 - t2) / 1e3, (t4 - t3) / 1e3, (t5 - t4) / 1e3,
             (t6 - t5) / 1e3, (t7 - t6) / 1e3, (t8 - t7) / 1e3);
}

static async_color_convert_handle_t s_cc;     // 2D-DMA: copias/recortes/conversión por filas

static void bench_more(void)
{
    // 1) recorte 1920x1080 desde 2560x1440 con el 2D-DMA (sin bloques de 16x16)
    async_color_convert_request_t rq = {
        .src_buffer = s_dec, .src_stride = 2560, .src_height = 1440, .src_x = 320, .src_y = 180,
        .dst_buffer = s_comp, .dst_stride = 1920, .dst_height = 1080, .dst_x = 0, .dst_y = 0,
        .copy_width = 1920, .copy_height = 1080,
        .src_color_format = ESP_COLOR_FOURCC_RGB16, .dst_color_format = ESP_COLOR_FOURCC_RGB16,
    };
    int64_t t0 = esp_timer_get_time();
    esp_err_t e1 = esp_color_convert_blocking(s_cc, &rq, -1);
    int64_t t1 = esp_timer_get_time();
    // 2) zoom 1.5x: recorte 1280x720 -> 1920x1080 con el PPA
    srm(s_dec, 2560, 1440, 640, 360, 1280, 720, PPA_SRM_COLOR_MODE_RGB565, s_comp, s_comp_cap, 1920, 1080, 0, 0,
        PPA_SRM_COLOR_MODE_RGB565, 1.5f, false);
    esp_err_t e2 = ESP_OK;
    int64_t t2 = esp_timer_get_time();
    size_t a4 = P4_ALIGN(1920 * 1080 / 2, 64);
    uint8_t *fg = heap_caps_aligned_calloc(64, 1, a4, MALLOC_CAP_SPIRAM);
    ppa_blend_oper_config_t op = {
        .in_bg = { .buffer = s_comp, .pic_w = 1920, .pic_h = 1080, .block_w = 1920, .block_h = 1080,
                   .blend_cm = PPA_BLEND_COLOR_MODE_RGB565 },
        .in_fg = { .buffer = fg, .pic_w = 1920, .pic_h = 1080, .block_w = 1920, .block_h = 1080,
                   .blend_cm = PPA_BLEND_COLOR_MODE_A4 },
        .out = { .buffer = s_comp, .buffer_size = s_comp_cap, .pic_w = 1920, .pic_h = 1080,
                 .blend_cm = PPA_BLEND_COLOR_MODE_RGB565 },
        .fg_fix_rgb_val = { .r = 255, .g = 255, .b = 255 },
        .mode = PPA_TRANS_MODE_BLOCKING,
    };
    op.out.buffer = s_comp; op.out.buffer_size = s_comp_cap; op.out.blend_cm = PPA_BLEND_COLOR_MODE_RGB565;
    esp_err_t e3 = fg ? ppa_do_blend(s_blend, &op) : ESP_ERR_NO_MEM;
    int64_t t3 = esp_timer_get_time();
    int64_t th0 = esp_timer_get_time();
    h264_encode(1920, 1080, false);
    int64_t th1 = esp_timer_get_time();
    free(fg);
    ppa_fill_oper_config_t fo = {
        .out = { .buffer = s_comp, .buffer_size = s_comp_cap, .pic_w = 1920, .pic_h = 1080, .fill_cm = PPA_FILL_COLOR_MODE_RGB565 },
        .fill_block_w = 1920, .fill_block_h = 1080, .fill_argb_color = { .val = 0xFF000000 }, .mode = PPA_TRANS_MODE_BLOCKING,
    };
    ppa_do_fill(s_fill, &fo);
    int64_t t4 = esp_timer_get_time();
    rq.src_buffer = s_dec; rq.dst_buffer = s_comp;
    rq.src_x = 0; rq.src_y = 0;
    rq.copy_width = 1920; rq.copy_height = 1080;
    rq.dst_color_format = ESP_COLOR_FOURCC_BGR24;       // RGB565 -> RGB888 por 2D-DMA (para la IA)
    rq.dst_buffer = ai_buffer(0); rq.dst_stride = 640; rq.dst_height = 360; rq.copy_width = 640; rq.copy_height = 360;
    int64_t t5 = esp_timer_get_time();
    esp_err_t e5 = esp_color_convert_blocking(s_cc, &rq, -1);
    int64_t t6 = esp_timer_get_time();
    ESP_LOGI(TAG, "BENCH2 recorte 2D-DMA 1080p %.1f ms (%s) | PPA zoom 720p->1080p %.1f ms (%s) | mezcla->RGB565 %.1f ms (%s) | "
             "relleno 1080p %.1f ms | 2D-DMA 565->888 640x360 %.1f ms (%s) | H.264 1080p (con YUV) %.1f ms",
             (t1 - t0) / 1e3, esp_err_to_name(e1), (t2 - t1) / 1e3, esp_err_to_name(e2), (t3 - t2) / 1e3,
             esp_err_to_name(e3), (t4 - t3) / 1e3, (t6 - t5) / 1e3, esp_err_to_name(e5), (th1 - th0) / 1e3);
}

static void video_task(void *arg)
{
    make_bench_jpeg();
    bench_engines();
    async_color_convert_config_t ccc = { .backlog = 4 };
    esp_async_color_convert_install_dma2d(&ccc, &s_cc);
    if (s_cc) bench_more();
    int64_t last_real = 0, last_bench = 0;
    int cur_w = 0, cur_h = 0;
    while (1) {
        int out_w = 1920, out_h = 1080;
        if (uvc_streaming()) uvc_size(&out_w, &out_h);
        if (out_w != cur_w) {
            hud_init(out_w);
            cur_w = out_w; cur_h = out_h;
        }
        jpeg_in_t *in = ingest_get(pdMS_TO_TICKS(g_cfg.max_mode ? 0 : 20));
        int w, h;
        if (in) {
            bool ok = decode(in->buf, in->len, &w, &h);
            ingest_release(in);
            if (ok) {
                process_frame(w, h, true, true, out_w, out_h);
                last_real = esp_timer_get_time();
            }
            continue;
        }
        int64_t now = esp_timer_get_time();
        bool pc_idle = now - last_real > 1000000;
        // sin PC: patrón de prueba a 30 fps. Modo máximo: patrón a toda velocidad entre cuadros reales.
        if (s_bench_jpeg && (g_cfg.max_mode || (pc_idle && now - last_bench > 33000))) {
            last_bench = now;
            if (decode(s_bench_jpeg, s_bench_len, &w, &h)) {
                process_frame(w, h, false, pc_idle, out_w, out_h);
            }
        }
    }
}

void pipeline_early_init(void)
{
    // el H.264 pide 135 KB de RAM interna contigua: se crea antes que todo lo demás
    g_cfg.h264_kbps = g_cfg.h264_kbps ? g_cfg.h264_kbps : 4000;
    s_h264_kbps = -1;
    esp_h264_enc_cfg_hw_t cfg = {
        .pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY,
        .gop = 30,
        .fps = 30,
        .res = { .width = 1920, .height = 1080 },
        .rc = { .bitrate = g_cfg.h264_kbps * 1000, .qp_min = 18, .qp_max = 38 },
    };
    if (esp_h264_enc_hw_new(&cfg, &s_h264) != ESP_H264_ERR_OK || esp_h264_enc_open(s_h264) != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "no se pudo crear el H.264 1920x1080");
        s_h264 = NULL;
        return;
    }
    esp_h264_enc_hw_get_param_hd(s_h264, &s_h264p);
    esp_h264_enc_roi_cfg_t roi = { .roi_mode = ESP_H264_ROI_MODE_DELTA_QP, .none_roi_delta_qp = 3 };
    esp_h264_enc_hw_cfg_roi(s_h264p, roi);
    esp_h264_enc_mv_cfg_t mv = { .mv_mode = ESP_H264_MVM_MODE_P16X16, .mv_fmt = ESP_H264_MVM_FMT_PART };
    esp_h264_enc_hw_cfg_mv(s_h264p, mv);
    s_mv_cap = (1920 / 16) * (P4_ALIGN(1080, 16) / 16) * sizeof(esp_h264_enc_mv_data_t);
    s_mv = heap_caps_aligned_calloc(64, 1, P4_ALIGN(s_mv_cap, 64), MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    s_h264_w = 1920; s_h264_h = 1080; s_h264_kbps = g_cfg.h264_kbps;
    ESP_LOGI(TAG, "H.264 1920x1080 listo a %d kbps (ROI en caras + vectores de movimiento)", g_cfg.h264_kbps);
}

void pipeline_start(void)
{
    jpeg_decode_engine_cfg_t dcfg = { .intr_priority = 0, .timeout_ms = 200 };
    ESP_ERROR_CHECK(jpeg_new_decoder_engine(&dcfg, &s_jdec));
    jpeg_encode_engine_cfg_t ecfg = { .intr_priority = 0, .timeout_ms = 200 };
    ESP_ERROR_CHECK(jpeg_new_encoder_engine(&ecfg, &s_jenc));

    ppa_client_config_t pc = { .oper_type = PPA_OPERATION_SRM, .max_pending_trans_num = 1 };
    ESP_ERROR_CHECK(ppa_register_client(&pc, &s_srm));
    pc.oper_type = PPA_OPERATION_FILL;
    ESP_ERROR_CHECK(ppa_register_client(&pc, &s_fill));
    pc.oper_type = PPA_OPERATION_BLEND;
    ESP_ERROR_CHECK(ppa_register_client(&pc, &s_blend));

    jpeg_decode_memory_alloc_cfg_t dm = { .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER };
    s_dec = jpeg_alloc_decoder_mem(SRC_MAX_W * SRC_MAX_H * 2, &dm, &s_dec_cap);
    s_comp_cap = P4_ALIGN(OUT_MAX_W * OUT_MAX_H * 2, 64);
    s_comp = alloc64(s_comp_cap);
    s_yuv_cap = P4_ALIGN(OUT_MAX_W * OUT_MAX_H * 3 / 2, 64);
    s_yuv = alloc64(s_yuv_cap);
    jpeg_encode_memory_alloc_cfg_t em = { .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER };
    for (int i = 0; i < N_JOUT; i++) s_jout[i] = jpeg_alloc_encoder_mem(640 * 1024, &em, &s_jout_cap);
    s_hout_cap = 640 * 1024;
    uint32_t act = 0;
    s_hout = esp_h264_aligned_calloc(64, 1, s_hout_cap, &act, ESP_H264_MEM_SPIRAM);
    for (int i = 0; i < N_HOUT; i++) s_hq[i] = heap_caps_malloc(s_hout_cap, MALLOC_CAP_SPIRAM);
    s_ring = heap_caps_malloc(RING_CAP, MALLOC_CAP_SPIRAM);
    s_rf = heap_caps_calloc(RF_MAX, sizeof(rframe_t), MALLOC_CAP_SPIRAM);
    s_jmux = xSemaphoreCreateMutex();
    s_ring_mux = xSemaphoreCreateMutex();
    if (!s_dec || !s_comp || !s_yuv || !s_jout[N_JOUT - 1] || !s_hout || !s_ring || !s_rf || !s_hq[N_HOUT - 1]) {
        ESP_LOGE(TAG, "falta memoria: dec=%p comp=%p yuv=%p jout=%p hout=%p ring=%p rf=%p hq=%p, PSRAM libre %u KB",
                 s_dec, s_comp, s_yuv, s_jout[N_JOUT - 1], s_hout, s_ring, s_rf, s_hq[N_HOUT - 1],
                 (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
        abort();
    }
    ESP_LOGI(TAG, "tubería lista: entrada hasta %dx%d, salida hasta %dx%d; PSRAM libre %u KB (bloque mayor %u KB)",
             SRC_MAX_W, SRC_MAX_H, OUT_MAX_W, 1080, (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
             (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM) / 1024));
    xTaskCreatePinnedToCore(video_task, "video", 12288, NULL, 14, NULL, 0);
}

float pipeline_motion_pct(void) { return s_motion_pct; }
