// Internet del P4 a través del cable USB: pide al PC un túnel TCP (ingest.c) y encima hace
// HTTPS él mismo con mbedTLS, que en el P4 usa los aceleradores de hardware (SHA, AES, ECC, RSA
// y el generador de números aleatorios). Trae el clima de Talca, el dólar y la UF, y la hora.
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "mbedtls/ssl.h"
#include "esp_crt_bundle.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "secrets.h"
#include "app.h"

static const char *TAG = "inet";

#define TZ_CHILE "<-04>4<-03>,M9.1.6/24,M4.1.6/24"
#define TAL_LAT  "-35.4264"
#define TAL_LON  "-71.6554"

static char s_wx[96], s_money[96];
static bool s_ok;
static int  s_tls_ms, s_fetches;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

void inet_status(char *wx, size_t wl, char *money, size_t ml, bool *ok, int *tls_ms, int *fetches)
{
    portENTER_CRITICAL(&s_mux);
    strlcpy(wx, s_wx, wl);
    strlcpy(money, s_money, ml);
    *ok = s_ok;
    *tls_ms = s_tls_ms;
    *fetches = s_fetches;
    portEXIT_CRITICAL(&s_mux);
}

static int bio_send(void *ctx, const unsigned char *buf, size_t len)
{
    int fd = *(int *)ctx;
    int r = send(fd, buf, len, 0);
    if (r < 0) return (errno == EAGAIN || errno == EWOULDBLOCK) ? MBEDTLS_ERR_SSL_WANT_WRITE : -0x004E;
    return r;
}

static int bio_recv(void *ctx, unsigned char *buf, size_t len)
{
    int fd = *(int *)ctx;
    int r = recv(fd, buf, len, 0);
    if (r < 0) return (errno == EAGAIN || errno == EWOULDBLOCK) ? MBEDTLS_ERR_SSL_WANT_READ : -0x004C;
    return r;
}

static time_t days_from_civil(int y, unsigned m, unsigned d)
{
    y -= m <= 2;
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return (time_t)era * 146097 + (time_t)doe - 719468;
}

// "Date: Thu, 25 Sep 2026 06:12:33 GMT" -> reloj del sistema
static void set_time_from_header(const char *resp)
{
    const char *p = strstr(resp, "\r\nDate: ");
    if (!p) p = strstr(resp, "\r\ndate: ");
    if (!p) return;
    char mon[4] = "";
    int d, y, hh, mm, ss;
    if (sscanf(p + 8, "%*3s, %d %3s %d %d:%d:%d", &d, mon, &y, &hh, &mm, &ss) != 6) return;
    static const char *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    const char *mp = strstr(months, mon);
    if (!mp) return;
    int m = (int)(mp - months) / 3 + 1;
    time_t t = days_from_civil(y, m, d) * 86400 + hh * 3600 + mm * 60 + ss;
    struct timeval tv = { .tv_sec = t, .tv_usec = 0 };
    settimeofday(&tv, NULL);
}

// GET https://host/path a través del túnel. Devuelve bytes leídos (cabeceras + cuerpo) o -1.
static int https_get(const char *host, const char *path, char *out, int cap)
{
    int fd = tunnel_open(host, 443, 8000);
    if (fd < 0) return -1;

    mbedtls_ssl_context ssl;
    mbedtls_ssl_config conf;
    mbedtls_ssl_init(&ssl);
    mbedtls_ssl_config_init(&conf);
    int n = -1, ret;
    if (mbedtls_ssl_config_defaults(&conf, MBEDTLS_SSL_IS_CLIENT, MBEDTLS_SSL_TRANSPORT_STREAM,
                                    MBEDTLS_SSL_PRESET_DEFAULT) != 0) goto done;
    mbedtls_ssl_conf_authmode(&conf, MBEDTLS_SSL_VERIFY_REQUIRED);
    esp_crt_bundle_attach(&conf);
    if (mbedtls_ssl_setup(&ssl, &conf) != 0) goto done;
    mbedtls_ssl_set_hostname(&ssl, host);
    mbedtls_ssl_set_bio(&ssl, &fd, bio_send, bio_recv, NULL);

    int64_t t0 = esp_timer_get_time();
    while ((ret = mbedtls_ssl_handshake(&ssl)) != 0) {
        if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE) {
            ESP_LOGW(TAG, "TLS con %s falló: -0x%04X", host, -ret);
            goto done;
        }
    }
    int hs_ms = (int)((esp_timer_get_time() - t0) / 1000);

    char req[512];
    int rl = snprintf(req, sizeof(req),
                      "GET %s HTTP/1.1\r\nHost: %s\r\nUser-Agent: Mozilla/5.0 (CerebroP4)\r\n"
                      "Accept: application/json\r\nConnection: close\r\n\r\n", path, host);
    for (int w = 0; w < rl;) {
        ret = mbedtls_ssl_write(&ssl, (const unsigned char *)req + w, rl - w);
        if (ret <= 0) goto done;
        w += ret;
    }
    n = 0;
    while (n < cap - 1) {
        ret = mbedtls_ssl_read(&ssl, (unsigned char *)out + n, cap - 1 - n);
        if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE) continue;
        if (ret <= 0) break;
        n += ret;
    }
    out[n] = 0;
    portENTER_CRITICAL(&s_mux);
    s_tls_ms = hs_ms;
    s_fetches++;
    portEXIT_CRITICAL(&s_mux);
    mbedtls_ssl_close_notify(&ssl);
done:
    mbedtls_ssl_free(&ssl);
    mbedtls_ssl_config_free(&conf);
    close(fd);
    return n;
}

static bool json_num_after(const char *s, const char *key_after, const char *key, double *v)
{
    const char *p = key_after ? strstr(s, key_after) : s;
    if (!p) return false;
    p = strstr(p, key);
    if (!p) return false;
    *v = strtod(p + strlen(key), NULL);
    return true;
}

static bool json_str(const char *s, const char *key, char *out, size_t cap)
{
    const char *p = strstr(s, key);
    if (!p) return false;
    p += strlen(key);
    size_t i = 0;
    while (*p && *p != '"' && i < cap - 1) out[i++] = *p++;
    out[i] = 0;
    return i > 0;
}

static void inet_task(void *arg)
{
    setenv("TZ", TZ_CHILE, 1);
    tzset();
    char *buf = malloc(24 * 1024);
    int64_t next_wx = 0, next_money = 0;
    while (1) {
        int64_t now = esp_timer_get_time();
        // clima de Talca
        if (now >= next_wx) {
            char path[256];
            snprintf(path, sizeof(path), "/data/2.5/weather?lat=%s&lon=%s&units=metric&lang=es&appid=%s",
                     TAL_LAT, TAL_LON, OWM_KEY);
            bool ok = false;
            if (https_get("api.openweathermap.org", path, buf, 24 * 1024) > 0) {
                set_time_from_header(buf);
                double t;
                char desc[48] = "";
                if (json_num_after(buf, "\"main\"", "\"temp\":", &t)) {
                    json_str(buf, "\"description\":\"", desc, sizeof(desc));
                    portENTER_CRITICAL(&s_mux);
                    snprintf(s_wx, sizeof(s_wx), "Talca %.0f° %s", t, desc);
                    portEXIT_CRITICAL(&s_mux);
                    ok = true;
                }
            }
            next_wx = esp_timer_get_time() + (ok ? 600000000LL : 30000000LL);   // 10 min, o reintento en 30 s
            if (ok) ESP_LOGI(TAG, "clima OK (TLS por hardware en %d ms): %s", s_tls_ms, s_wx);
        }
        // dólar y UF
        if (esp_timer_get_time() >= next_money) {
            bool ok = false;
            if (https_get("mindicador.cl", "/api", buf, 24 * 1024) > 0) {
                set_time_from_header(buf);
                double usd = 0, uf = 0;
                bool a = json_num_after(buf, "\"dolar\"", "\"valor\":", &usd);
                bool b = json_num_after(buf, "\"uf\"", "\"valor\":", &uf);
                if (a || b) {
                    portENTER_CRITICAL(&s_mux);
                    snprintf(s_money, sizeof(s_money), "USD %.0f · UF %.0f", usd, uf);
                    portEXIT_CRITICAL(&s_mux);
                    ok = true;
                }
            }
            next_money = esp_timer_get_time() + (ok ? 600000000LL : 30000000LL);
            if (ok) ESP_LOGI(TAG, "indicadores OK (TLS por hardware en %d ms): %s", s_tls_ms, s_money);
        }
        s_ok = s_wx[0] || s_money[0];
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void inet_start(void)
{
    xTaskCreatePinnedToCore(inet_task, "inet", 12288, NULL, 5, NULL, 0);
}
