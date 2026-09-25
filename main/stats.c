// Medidor de uso: % del tiempo que cada acelerador pasa ocupado, % de CPU por núcleo,
// temperatura del chip, cuadros por segundo y tráfico de red. Ventanas de 1 segundo.
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/temperature_sensor.h"
#include "app.h"

static const char *TAG = "stats";

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static int64_t s_busy_acc[ENG_COUNT];
static int64_t s_busy_last_end[ENG_COUNT];
static float   s_eng_pct[ENG_COUNT];
static const char *s_eng_name[ENG_COUNT] = { "jpeg_dec", "jpeg_enc", "ppa", "h264", "usb", "dma2d" };

static uint32_t s_fin, s_fout, s_ffill;
static uint64_t s_bytes_in, s_net_rx, s_net_tx;
static float s_fps_fill;
static float s_fps_in, s_fps_out, s_kbps_in, s_net_rx_kbps, s_net_tx_kbps;
static float s_cpu[2];
static float s_temp = -1;
static temperature_sensor_handle_t s_tsens;

void stats_busy(eng_t e, int64_t t0, int64_t t1)
{
    portENTER_CRITICAL(&s_mux);
    // unión de intervalos: si dos tareas usan el mismo motor, no se cuenta doble
    if (t0 < s_busy_last_end[e]) {
        t0 = s_busy_last_end[e];
    }
    if (t1 > t0) {
        s_busy_acc[e] += t1 - t0;
        s_busy_last_end[e] = t1;
    }
    portEXIT_CRITICAL(&s_mux);
}

void stats_frame_in(size_t bytes) { portENTER_CRITICAL(&s_mux); s_fin++; s_bytes_in += bytes; portEXIT_CRITICAL(&s_mux); }
void stats_frame_out(void)        { portENTER_CRITICAL(&s_mux); s_fout++; portEXIT_CRITICAL(&s_mux); }
void stats_filler(void)           { portENTER_CRITICAL(&s_mux); s_ffill++; portEXIT_CRITICAL(&s_mux); }
void stats_net(size_t rx, size_t tx) { portENTER_CRITICAL(&s_mux); s_net_rx += rx; s_net_tx += tx; portEXIT_CRITICAL(&s_mux); }

float stats_eng_pct(eng_t e) { return s_eng_pct[e]; }
float stats_cpu_pct(int core) { return s_cpu[core & 1]; }
float stats_temp(void)       { return s_temp; }
float stats_fps_in(void)     { return s_fps_in; }
float stats_fps_out(void)    { return s_fps_out; }

// ---------------- CPU por núcleo: 100% - tiempo de la tarea IDLE ----------------
static uint64_t s_idle_prev[2];
static int64_t  s_t_prev;

static void cpu_sample(int64_t now, int64_t dt)
{
    for (int c = 0; c < 2; c++) {
        uint64_t rt = ulTaskGetIdleRunTimeCounterForCore(c);
        uint64_t d = rt - s_idle_prev[c];
        s_idle_prev[c] = rt;
        float idle_pct = dt > 0 ? (100.0f * (float)d / (float)dt) : 0;
        if (idle_pct > 100) idle_pct = 100;
        s_cpu[c] = 100.0f - idle_pct;
    }
}

static void stats_task(void *arg)
{
    s_t_prev = esp_timer_get_time();
    uint32_t fin0 = 0, fout0 = 0, ffill0 = 0;
    uint64_t bin0 = 0, rx0 = 0, tx0 = 0;
    for (int c = 0; c < 2; c++) s_idle_prev[c] = ulTaskGetIdleRunTimeCounterForCore(c);
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        int64_t now = esp_timer_get_time();
        int64_t dt = now - s_t_prev;
        s_t_prev = now;
        portENTER_CRITICAL(&s_mux);
        for (int e = 0; e < ENG_COUNT; e++) {
            float p = 100.0f * (float)s_busy_acc[e] / (float)dt;
            s_eng_pct[e] = p > 100 ? 100 : p;
            s_busy_acc[e] = 0;
        }
        uint32_t fin = s_fin, fout = s_fout, ffill = s_ffill;
        uint64_t bin = s_bytes_in, rx = s_net_rx, tx = s_net_tx;
        portEXIT_CRITICAL(&s_mux);
        float sec = dt / 1e6f;
        s_fps_in = (fin - fin0) / sec;
        s_fps_out = (fout - fout0) / sec;
        s_fps_fill = (ffill - ffill0) / sec;
        s_kbps_in = (bin - bin0) * 8 / 1000.0f / sec;
        s_net_rx_kbps = (rx - rx0) * 8 / 1000.0f / sec;
        s_net_tx_kbps = (tx - tx0) * 8 / 1000.0f / sec;
        fin0 = fin; fout0 = fout; ffill0 = ffill; bin0 = bin; rx0 = rx; tx0 = tx;
        cpu_sample(now, dt);
        if (s_tsens) {
            float t;
            if (temperature_sensor_get_celsius(s_tsens, &t) == ESP_OK) s_temp = t;
        }
        static int k;
        if (++k % 5 == 0) {
            ESP_LOGI(TAG, "salida %.1f fps (relleno %.1f) | CPU %.0f/%.0f%% | JPEGdec %.0f%% JPEGenc %.0f%% PPA %.0f%% H264 %.0f%% USB %.0f%% | %.1f C | PSRAM libre %u KB",
                     s_fps_out, s_fps_fill, s_cpu[0], s_cpu[1], s_eng_pct[ENG_JDEC], s_eng_pct[ENG_JENC], s_eng_pct[ENG_PPA],
                     s_eng_pct[ENG_H264], s_eng_pct[ENG_USB], s_temp, (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
        }
    }
}

int stats_json(char *b, size_t cap)
{
    uint32_t primes = 0, last = 0, loops = 0;
    lpcore_stats(&primes, &last, &loops);
    float ffps, fms, cfps, cms;
    ai_stats(&ffps, &fms, &cfps, &cms);
    det_list_t dl;
    ai_get_dets(&dl);
    int faces = 0, cats = 0, people = 0;
    for (int i = 0; i < dl.n; i++) { if (dl.d[i].cls == 0) faces++; else if (dl.d[i].cls == 1) cats++; else people++; }
    float pfps, pms;
    ai_stats_person(&pfps, &pms);
    char wx[96] = "", money[96] = "";
    bool iok = false; int tls_ms = 0, fetches = 0;
    inet_status(wx, sizeof(wx), money, sizeof(money), &iok, &tls_ms, &fetches);
    int uw, uh; uvc_size(&uw, &uh);

    int n = snprintf(b, cap,
        "{\"up\":%lld,\"temp\":%.1f,\"cpu\":[%.1f,%.1f],"
        "\"fps_in\":%.1f,\"fps_out\":%.1f,\"fps_fill\":%.1f,\"kbps_in\":%.0f,\"net_rx_kbps\":%.0f,\"net_tx_kbps\":%.0f,"
        "\"eng\":{",
        esp_timer_get_time() / 1000000, s_temp, s_cpu[0], s_cpu[1],
        s_fps_in, s_fps_out, s_fps_fill, s_kbps_in, s_net_rx_kbps, s_net_tx_kbps);
    for (int e = 0; e < ENG_COUNT && n < (int)cap; e++) {
        n += snprintf(b + n, cap - n, "%s\"%s\":%.1f", e ? "," : "", s_eng_name[e], s_eng_pct[e]);
    }
    n += snprintf(b + n, cap - n,
        "},\"ai\":{\"face_fps\":%.1f,\"face_ms\":%.1f,\"cat_fps\":%.1f,\"cat_ms\":%.1f,\"person_fps\":%.1f,\"person_ms\":%.1f,\"faces\":%d,\"cats\":%d,\"people\":%d,\"motion\":%.1f},"
        "\"lp\":{\"primes\":%lu,\"last\":%lu,\"loops\":%lu},"
        "\"uvc\":{\"on\":%s,\"fmt\":\"%s\",\"w\":%d,\"h\":%d,\"sent\":%lu},"
        "\"pc\":%s,\"usb\":%s,"
        "\"inet\":{\"ok\":%s,\"tls_ms\":%d,\"fetches\":%d,\"wx\":\"%s\",\"money\":\"%s\"},"
        "\"heap_int\":%u,\"heap_psram\":%u,"
        "\"cfg\":{\"autoframe\":%s,\"mirror\":%s,\"hud\":%s,\"boxes\":%s,\"max\":%s,\"q\":%d,\"zoom\":%.2f,\"kbps\":%d,\"h264\":%s,\"ia\":%d}}",
        ffps, fms, cfps, cms, pfps, pms, faces, cats, people, pipeline_motion_pct(),
        (unsigned long)primes, (unsigned long)last, (unsigned long)loops,
        uvc_streaming() ? "true" : "false", uvc_format() == 2 ? "H.264" : "MJPEG", uw, uh, (unsigned long)uvc_frames_sent(),
        ingest_pc_connected() ? "true" : "false", usb_mounted() ? "true" : "false",
        iok ? "true" : "false", tls_ms, fetches, wx, money,
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL), (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
        g_cfg.autoframe ? "true" : "false", g_cfg.mirror ? "true" : "false", g_cfg.hud ? "true" : "false",
        g_cfg.boxes ? "true" : "false", g_cfg.max_mode ? "true" : "false", g_cfg.jpeg_q, g_cfg.zoom_max, g_cfg.h264_kbps,
        g_cfg.h264_on ? "true" : "false", g_cfg.ai_level);
    return n;
}

// ---------------- Registro (log) en RAM para verlo desde el dashboard ----------------
#define LOG_RING 8192
static char s_log[LOG_RING];
static size_t s_log_head;
static bool s_log_wrap;
static vprintf_like_t s_prev_vprintf;
static portMUX_TYPE s_log_mux = portMUX_INITIALIZER_UNLOCKED;

static int log_vprintf(const char *fmt, va_list ap)
{
    char line[256];
    va_list ap2;
    va_copy(ap2, ap);
    int n = vsnprintf(line, sizeof(line), fmt, ap2);
    va_end(ap2);
    if (n > 0) {
        if (n > (int)sizeof(line) - 1) n = sizeof(line) - 1;
        portENTER_CRITICAL(&s_log_mux);
        for (int i = 0; i < n; i++) {
            s_log[s_log_head++] = line[i];
            if (s_log_head == LOG_RING) { s_log_head = 0; s_log_wrap = true; }
        }
        portEXIT_CRITICAL(&s_log_mux);
    }
    return s_prev_vprintf ? s_prev_vprintf(fmt, ap) : n;
}

void log_ring_init(void)
{
    s_prev_vprintf = esp_log_set_vprintf(log_vprintf);
}

int log_ring_copy(char *dst, int cap)
{
    int n = 0;
    portENTER_CRITICAL(&s_log_mux);
    if (s_log_wrap) {
        for (size_t i = s_log_head; i < LOG_RING && n < cap - 1; i++) dst[n++] = s_log[i];
    }
    for (size_t i = 0; i < s_log_head && n < cap - 1; i++) dst[n++] = s_log[i];
    portEXIT_CRITICAL(&s_log_mux);
    dst[n] = 0;
    return n;
}

void stats_start(void)
{
    temperature_sensor_config_t tcfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 100);
    if (temperature_sensor_install(&tcfg, &s_tsens) != ESP_OK) {
        temperature_sensor_config_t t2 = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
        if (temperature_sensor_install(&t2, &s_tsens) != ESP_OK) {
            s_tsens = NULL;
        }
    }
    if (s_tsens) {
        temperature_sensor_enable(s_tsens);
    } else {
        ESP_LOGW(TAG, "sin sensor de temperatura");
    }
    xTaskCreatePinnedToCore(stats_task, "stats", 4096, NULL, 15, NULL, 0);
}
