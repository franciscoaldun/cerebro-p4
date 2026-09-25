// Entrada de cuadros y túnel a internet. Todas las conexiones las ABRE el PC (así el
// firewall de Windows no pide permisos de administrador):
//   - puerto 5000: el PC empuja cuadros JPEG de su cámara  ->  [ 'P4JF' | largo | seq | flags ] + JPEG
//   - puerto 5001: el PC deja conexiones esperando; cuando el P4 necesita internet escribe
//                  "CONNECT host:puerto\n", el PC abre esa conexión y responde "OK\n".
#include <string.h>
#include <errno.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "freertos/queue.h"
#include "driver/jpeg_decode.h"
#include "app.h"

static const char *TAG = "ingest";

#define N_IN      3
#define IN_CAP    (1024 * 1024)     // 1 MB por cuadro JPEG (un 2560x1440 de calidad 85 pesa ~0,5 MB)
#define MAGIC     0x464A3450u       // "P4JF"

static jpeg_in_t     s_in[N_IN];
static QueueHandle_t s_free, s_full;
static volatile bool s_pc_frames;
static volatile int64_t s_last_rx;

jpeg_in_t *ingest_get(TickType_t wait)
{
    jpeg_in_t *f = NULL;
    if (xQueueReceive(s_full, &f, wait) == pdTRUE) {
        return f;
    }
    return NULL;
}

void ingest_release(jpeg_in_t *f)
{
    if (f) {
        xQueueSend(s_free, &f, 0);
    }
}

bool ingest_pc_connected(void)
{
    return s_pc_frames || (esp_timer_get_time() - s_last_rx) < 3000000;
}

static int recv_all(int fd, void *buf, size_t n)
{
    uint8_t *p = buf;
    while (n) {
        int r = recv(fd, p, n, 0);
        if (r <= 0) {
            return -1;
        }
        p += r;
        n -= r;
    }
    return 0;
}

static void frames_client(int fd)
{
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    s_pc_frames = true;
    ESP_LOGI(TAG, "el PC empezó a mandar cuadros");
    ota_mark_ok_if_pending();
    uint32_t hdr[4];
    while (1) {
        if (recv_all(fd, hdr, sizeof(hdr)) < 0 || hdr[0] != MAGIC) {
            break;
        }
        uint32_t len = hdr[1];
        jpeg_in_t *f = NULL;
        if (xQueueReceive(s_free, &f, 0) != pdTRUE) {
            // la tubería va atrasada: se bota el cuadro más viejo para no acumular retardo
            if (xQueueReceive(s_full, &f, 0) != pdTRUE) {
                xQueueReceive(s_free, &f, portMAX_DELAY);
            }
        }
        if (len > f->cap) {
            // cuadro demasiado grande: se descarta leyéndolo en trozos
            ESP_LOGW(TAG, "cuadro de %lu bytes, máximo %u", (unsigned long)len, (unsigned)f->cap);
            size_t left = len;
            while (left) {
                size_t k = left > f->cap ? f->cap : left;
                if (recv_all(fd, f->buf, k) < 0) { xQueueSend(s_free, &f, 0); goto out; }
                left -= k;
            }
            xQueueSend(s_free, &f, 0);
            continue;
        }
        if (recv_all(fd, f->buf, len) < 0) {
            xQueueSend(s_free, &f, 0);
            break;
        }
        f->len = len;
        f->seq = hdr[2];
        f->t_rx_us = esp_timer_get_time();
        s_last_rx = f->t_rx_us;
        stats_frame_in(len);
        xQueueSend(s_full, &f, 0);
    }
out:
    s_pc_frames = false;
    ESP_LOGW(TAG, "el PC dejó de mandar cuadros");
}

static int make_listener(int port)
{
    int s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    int one = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons(port), .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (bind(s, (struct sockaddr *)&a, sizeof(a)) < 0 || listen(s, 4) < 0) {
        ESP_LOGE(TAG, "no se pudo escuchar en %d", port);
        close(s);
        return -1;
    }
    return s;
}

static void frames_task(void *arg)
{
    int ls = make_listener(PORT_FRAMES);
    while (ls >= 0) {
        int fd = accept(ls, NULL, NULL);
        if (fd < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        frames_client(fd);
        close(fd);
    }
    vTaskDelete(NULL);
}

// ---------------- Túnel a internet ----------------
static QueueHandle_t s_pool;     // sockets que el PC dejó esperando

static void tunnel_task(void *arg)
{
    int ls = make_listener(PORT_TUNNEL);
    while (ls >= 0) {
        int fd = accept(ls, NULL, NULL);
        if (fd < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        s_last_rx = esp_timer_get_time();
        if (xQueueSend(s_pool, &fd, 0) != pdTRUE) {
            close(fd);     // ya hay suficientes esperando
        }
    }
    vTaskDelete(NULL);
}

int tunnel_open(const char *host, int port, int timeout_ms)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline) {
        int fd;
        if (xQueueReceive(s_pool, &fd, pdMS_TO_TICKS(500)) != pdTRUE) {
            continue;
        }
        struct timeval tv = { .tv_sec = 10, .tv_usec = 0 };
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        char line[160];
        int n = snprintf(line, sizeof(line), "CONNECT %s:%d\n", host, port);
        if (send(fd, line, n, 0) != n) {
            close(fd);
            continue;
        }
        // respuesta: "OK\n" o "ERR ...\n"
        int k = 0;
        char c;
        while (k < (int)sizeof(line) - 1 && recv(fd, &c, 1, 0) == 1) {
            if (c == '\n') break;
            line[k++] = c;
        }
        line[k] = 0;
        if (strncmp(line, "OK", 2) == 0) {
            return fd;
        }
        ESP_LOGW(TAG, "túnel a %s:%d falló: '%s'", host, port, line);
        close(fd);
        return -1;
    }
    return -1;
}

void ingest_start(void)
{
    s_free = xQueueCreate(N_IN, sizeof(jpeg_in_t *));
    s_full = xQueueCreate(N_IN, sizeof(jpeg_in_t *));
    s_pool = xQueueCreate(8, sizeof(int));
    jpeg_decode_memory_alloc_cfg_t mc = { .buffer_direction = JPEG_DEC_ALLOC_INPUT_BUFFER };
    for (int i = 0; i < N_IN; i++) {
        size_t got = 0;
        s_in[i].buf = jpeg_alloc_decoder_mem(IN_CAP, &mc, &got);
        s_in[i].cap = got;
        jpeg_in_t *p = &s_in[i];
        xQueueSend(s_free, &p, 0);
    }
    xTaskCreatePinnedToCore(frames_task, "frames", 6144, NULL, 11, NULL, 0);
    xTaskCreatePinnedToCore(tunnel_task, "tunnel", 4096, NULL, 6, NULL, 0);
}
