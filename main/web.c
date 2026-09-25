// Servidor web del P4 (por la red USB): http://192.168.7.1
//   /              dashboard            /api/stats   números en vivo (JSON)
//   /api/cfg?...   cambiar opciones     /api/log     registro
//   /snap.jpg      foto actual          :81/stream   video MJPEG en vivo
//   /replay.h264?s=20  últimos segundos grabados      POST /ota  actualizar firmware
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app.h"

static const char *TAG = "web";

extern const char dash_start[] asm("_binary_dashboard_html_start");
extern const char dash_end[]   asm("_binary_dashboard_html_end");

static esp_err_t h_root(httpd_req_t *r)
{
    httpd_resp_set_type(r, "text/html; charset=utf-8");
    return httpd_resp_send(r, dash_start, dash_end - dash_start);
}

static esp_err_t h_stats(httpd_req_t *r)
{
    char *b = malloc(4096);
    int n = stats_json(b, 4096);
    httpd_resp_set_type(r, "application/json");
    httpd_resp_set_hdr(r, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(r, "Cache-Control", "no-store");
    esp_err_t e = httpd_resp_send(r, b, n);
    free(b);
    ota_mark_ok_if_pending();
    return e;
}

static esp_err_t h_log(httpd_req_t *r)
{
    char *b = malloc(8200);
    int n = log_ring_copy(b, 8200);
    httpd_resp_set_type(r, "text/plain; charset=utf-8");
    esp_err_t e = httpd_resp_send(r, b, n);
    free(b);
    return e;
}

static esp_err_t h_tasks(httpd_req_t *r)
{
    char *b = malloc(4096);
    vTaskGetRunTimeStats(b);
    httpd_resp_set_type(r, "text/plain; charset=utf-8");
    esp_err_t e = httpd_resp_sendstr(r, b);
    free(b);
    return e;
}

static esp_err_t h_cfg(httpd_req_t *r)
{
    char q[256], v[16];
    if (httpd_req_get_url_query_str(r, q, sizeof(q)) == ESP_OK) {
        if (httpd_query_key_value(q, "autoframe", v, sizeof(v)) == ESP_OK) g_cfg.autoframe = atoi(v);
        if (httpd_query_key_value(q, "mirror", v, sizeof(v)) == ESP_OK) g_cfg.mirror = atoi(v);
        if (httpd_query_key_value(q, "hud", v, sizeof(v)) == ESP_OK) g_cfg.hud = atoi(v);
        if (httpd_query_key_value(q, "boxes", v, sizeof(v)) == ESP_OK) g_cfg.boxes = atoi(v);
        if (httpd_query_key_value(q, "max", v, sizeof(v)) == ESP_OK) g_cfg.max_mode = atoi(v);
        if (httpd_query_key_value(q, "h264", v, sizeof(v)) == ESP_OK) g_cfg.h264_on = atoi(v);
        if (httpd_query_key_value(q, "ia", v, sizeof(v)) == ESP_OK) {
            int x = atoi(v);
            g_cfg.ai_level = x < 0 ? 0 : x > 3 ? 3 : x;
        }
        if (httpd_query_key_value(q, "perfil", v, sizeof(v)) == ESP_OK) {
            if (strcmp(v, "fluido") == 0) {          // más cuadros por segundo: sin grabación, sólo caras
                g_cfg.h264_on = false; g_cfg.ai_level = 1; g_cfg.max_mode = false;
            } else if (strcmp(v, "maximo") == 0) {   // todo el silicio: H.264 en cada cuadro, 3 IA, relleno
                g_cfg.h264_on = true; g_cfg.ai_level = 3; g_cfg.max_mode = true;
            } else {                                 // equilibrado (el de fábrica)
                g_cfg.h264_on = true; g_cfg.ai_level = 3; g_cfg.max_mode = false;
            }
        }
        if (httpd_query_key_value(q, "q", v, sizeof(v)) == ESP_OK) {
            int x = atoi(v);
            g_cfg.jpeg_q = x < 40 ? 40 : x > 95 ? 95 : x;
        }
        if (httpd_query_key_value(q, "zoom", v, sizeof(v)) == ESP_OK) {
            float z = atof(v);
            g_cfg.zoom_max = z < 1 ? 1 : z > 3 ? 3 : z;
        }
        if (httpd_query_key_value(q, "kbps", v, sizeof(v)) == ESP_OK) {
            int k = atoi(v);
            g_cfg.h264_kbps = k < 500 ? 500 : k > 20000 ? 20000 : k;
        }
    }
    return h_stats(r);
}

static esp_err_t h_snap(httpd_req_t *r)
{
    size_t cap = 1024 * 1024;
    uint8_t *b = heap_caps_malloc(cap, MALLOC_CAP_SPIRAM);
    uint32_t seq = 0;
    size_t n = pipeline_copy_jpeg(b, cap, &seq, pdMS_TO_TICKS(2000));
    httpd_resp_set_type(r, "image/jpeg");
    httpd_resp_set_hdr(r, "Cache-Control", "no-store");
    esp_err_t e = n ? httpd_resp_send(r, (const char *)b, n) : httpd_resp_send_500(r);
    free(b);
    return e;
}

static int emit_chunk(void *ctx, const uint8_t *p, size_t n)
{
    return httpd_resp_send_chunk((httpd_req_t *)ctx, (const char *)p, n) == ESP_OK ? 0 : -1;
}

static esp_err_t h_replay(httpd_req_t *r)
{
    char q[64], v[8];
    int sec = 20;
    if (httpd_req_get_url_query_str(r, q, sizeof(q)) == ESP_OK &&
        httpd_query_key_value(q, "s", v, sizeof(v)) == ESP_OK) sec = atoi(v);
    if (sec < 2) sec = 2;
    if (sec > 60) sec = 60;
    httpd_resp_set_type(r, "video/h264");
    httpd_resp_set_hdr(r, "Content-Disposition", "attachment; filename=\"repeticion-p4.h264\"");
    int n = replay_stream(emit_chunk, r, sec);
    httpd_resp_send_chunk(r, NULL, 0);
    ESP_LOGI(TAG, "repetición de %d s enviada (%d KB)", sec, n / 1024);
    return ESP_OK;
}

// ---------------- Actualización de firmware por la red USB ----------------
static esp_err_t h_ota(httpd_req_t *r)
{
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    esp_ota_handle_t h;
    if (!part || esp_ota_begin(part, OTA_WITH_SEQUENTIAL_WRITES, &h) != ESP_OK) {
        return httpd_resp_send_500(r);
    }
    led_state(LED_OTA);
    char *buf = malloc(16384);
    int left = r->content_len, got = 0;
    ESP_LOGI(TAG, "OTA: recibiendo %d KB", left / 1024);
    while (left > 0) {
        int n = httpd_req_recv(r, buf, left > 16384 ? 16384 : left);
        if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (n <= 0 || esp_ota_write(h, buf, n) != ESP_OK) {
            free(buf);
            esp_ota_abort(h);
            led_state(LED_ERR);
            return httpd_resp_send_500(r);
        }
        left -= n;
        got += n;
    }
    free(buf);
    if (esp_ota_end(h) != ESP_OK || esp_ota_set_boot_partition(part) != ESP_OK) {
        led_state(LED_ERR);
        return httpd_resp_send_500(r);
    }
    httpd_resp_sendstr(r, "OK, reiniciando con el firmware nuevo\n");
    ESP_LOGI(TAG, "OTA OK (%d KB), reiniciando", got / 1024);
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

// Si arrancó un firmware nuevo, se confirma sólo cuando el PC logra hablarle por la red.
// Si no lo logra en 3 minutos, se reinicia y el bootloader vuelve al anterior.
static bool s_pending;
void ota_mark_ok_if_pending(void)
{
    if (s_pending) {
        s_pending = false;
        esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGI(TAG, "firmware nuevo confirmado");
    }
}

static void ota_guard_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(180000));
    if (s_pending) {
        ESP_LOGE(TAG, "el firmware nuevo no logró conectarse; vuelvo al anterior");
        esp_ota_mark_app_invalid_rollback_and_reboot();
    }
    vTaskDelete(NULL);
}

// ---------------- Video en vivo (puerto 81, una tarea por espectador) ----------------
#define BOUNDARY "cerebrop4frame"

static void stream_task(void *arg)
{
    httpd_req_t *r = arg;
    size_t cap = 1024 * 1024;
    uint8_t *b = heap_caps_malloc(cap, MALLOC_CAP_SPIRAM);
    httpd_resp_set_type(r, "multipart/x-mixed-replace; boundary=" BOUNDARY);
    httpd_resp_set_hdr(r, "Access-Control-Allow-Origin", "*");
    uint32_t seq = 0;
    char hdr[96];
    while (b) {
        size_t n = pipeline_copy_jpeg(b, cap, &seq, pdMS_TO_TICKS(3000));
        if (!n) continue;
        int hl = snprintf(hdr, sizeof(hdr), "\r\n--" BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                          (unsigned)n);
        if (httpd_resp_send_chunk(r, hdr, hl) != ESP_OK || httpd_resp_send_chunk(r, (const char *)b, n) != ESP_OK) {
            break;
        }
    }
    free(b);
    httpd_req_async_handler_complete(r);
    vTaskDelete(NULL);
}

static esp_err_t h_stream(httpd_req_t *r)
{
    httpd_req_t *copy = NULL;
    if (httpd_req_async_handler_begin(r, &copy) != ESP_OK) return ESP_FAIL;
    if (xTaskCreatePinnedToCore(stream_task, "stream", 6144, copy, 6, NULL, 0) != pdPASS) {
        httpd_req_async_handler_complete(copy);
        return ESP_FAIL;
    }
    return ESP_OK;
}

void web_start(void)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(run, &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY) {
        s_pending = true;
        xTaskCreate(ota_guard_task, "ota_guard", 3072, NULL, 5, NULL);
        ESP_LOGW(TAG, "firmware nuevo a prueba: se confirma cuando el PC se conecte");
    }

    httpd_config_t c = HTTPD_DEFAULT_CONFIG();
    c.server_port = PORT_WEB;
    c.ctrl_port = 32768;
    c.max_uri_handlers = 12;
    c.stack_size = 8192;
    c.core_id = 0;
    c.lru_purge_enable = true;
    c.task_priority = 12;          // la web nunca se queda sin turno aunque el video vaya a full
    c.recv_wait_timeout = 30;
    c.send_wait_timeout = 30;
    httpd_handle_t s = NULL;
    if (httpd_start(&s, &c) == ESP_OK) {
        httpd_uri_t u[] = {
            { .uri = "/", .method = HTTP_GET, .handler = h_root },
            { .uri = "/api/stats", .method = HTTP_GET, .handler = h_stats },
            { .uri = "/api/cfg", .method = HTTP_GET, .handler = h_cfg },
            { .uri = "/api/log", .method = HTTP_GET, .handler = h_log },
            { .uri = "/api/tasks", .method = HTTP_GET, .handler = h_tasks },
            { .uri = "/snap.jpg", .method = HTTP_GET, .handler = h_snap },
            { .uri = "/replay.h264", .method = HTTP_GET, .handler = h_replay },
            { .uri = "/ota", .method = HTTP_POST, .handler = h_ota },
        };
        for (size_t i = 0; i < sizeof(u) / sizeof(u[0]); i++) httpd_register_uri_handler(s, &u[i]);
    }

    httpd_config_t c2 = HTTPD_DEFAULT_CONFIG();
    c2.server_port = PORT_STREAM;
    c2.ctrl_port = 32769;
    c2.max_open_sockets = 4;
    c2.core_id = 0;
    c2.lru_purge_enable = true;
    c2.task_priority = 7;
    httpd_handle_t s2 = NULL;
    if (httpd_start(&s2, &c2) == ESP_OK) {
        httpd_uri_t us = { .uri = "/stream", .method = HTTP_GET, .handler = h_stream };
        httpd_register_uri_handler(s2, &us);
    }
    ESP_LOGI(TAG, "dashboard en http://" P4_IP_STR "  (video en :81/stream)");
}
