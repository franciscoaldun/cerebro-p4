// Barra de información que se superpone al video (ARGB8888, la mezcla la hace el PPA).
#include <string.h>
#include <stdio.h>
#include <time.h>
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "font.h"
#include "app.h"

#define HUD_H 44

static uint8_t *s_buf;
static int s_w;
static size_t s_cap;

uint8_t *hud_buf(int *w, int *h)
{
    *w = s_w;
    *h = HUD_H;
    return s_buf;
}

void hud_init(int w)
{
    s_w = w;
    s_cap = P4_ALIGN((size_t)w * HUD_H * 4, 64);
    s_buf = heap_caps_aligned_calloc(64, 1, s_cap, MALLOC_CAP_SPIRAM);
}

static int glyph_index(uint32_t cp)
{
    if (cp >= 32 && cp < 127) return (int)cp - 32;
    for (int i = 95; i < FONT_N; i++) {
        if (font_cp[i] == cp) return i;
    }
    return '?' - 32;
}

// decodifica un carácter UTF-8
static uint32_t utf8_next(const char **ps)
{
    const uint8_t *s = (const uint8_t *)*ps;
    uint32_t cp = *s++;
    if (cp >= 0xE0 && s[0] && s[1]) { cp = ((cp & 0x0F) << 12) | ((s[0] & 0x3F) << 6) | (s[1] & 0x3F); s += 2; }
    else if (cp >= 0xC0 && s[0]) { cp = ((cp & 0x1F) << 6) | (s[0] & 0x3F); s += 1; }
    *ps = (const char *)s;
    return cp;
}

void hud_text_width(const char *s, int *px)
{
    int n = 0;
    while (*s) { utf8_next(&s); n++; }
    *px = n * FONT_W;
}

// pinta texto (color rgb) sobre la barra, mezclando el alfa del glifo
static void draw_text(int x, int y, const char *s, uint8_t r, uint8_t g, uint8_t b)
{
    while (*s) {
        uint32_t cp = utf8_next(&s);
        const uint8_t *gl = font_a8[glyph_index(cp)];
        for (int gy = 0; gy < FONT_H; gy++) {
            int py = y + gy;
            if (py < 0 || py >= HUD_H) continue;
            uint8_t *row = s_buf + ((size_t)py * s_w) * 4;
            for (int gx = 0; gx < FONT_W; gx++) {
                int px = x + gx;
                if (px < 0 || px >= s_w) continue;
                uint8_t a = gl[gy * FONT_W + gx];
                if (!a) continue;
                uint8_t *p = row + px * 4;          // memoria: B G R A
                p[0] = (uint8_t)((b * a + p[0] * (255 - a)) / 255);
                p[1] = (uint8_t)((g * a + p[1] * (255 - a)) / 255);
                p[2] = (uint8_t)((r * a + p[2] * (255 - a)) / 255);
                if (p[3] < a) p[3] = a;
            }
        }
        x += FONT_W;
    }
}

void hud_render(void)
{
    if (!s_buf) return;
    // fondo: franja oscura semitransparente con una línea cian abajo
    uint32_t *px = (uint32_t *)s_buf;
    for (int y = 0; y < HUD_H; y++) {
        uint32_t c = (y >= HUD_H - 2) ? 0xE000D8FFu : 0xA80A0E14u;   // ARGB
        for (int x = 0; x < s_w; x++) px[y * s_w + x] = c;
    }

    det_list_t dl;
    ai_get_dets(&dl);
    int faces = 0, cats = 0, people = 0;
    for (int i = 0; i < dl.n; i++) { if (dl.d[i].cls == 0) faces++; else if (dl.d[i].cls == 1) cats++; else people++; }

    char left[160], right[200], wx[96] = "", money[96] = "";
    bool ok = false; int tls = 0, fe = 0;
    inet_status(wx, sizeof(wx), money, sizeof(money), &ok, &tls, &fe);

    snprintf(left, sizeof(left), "CEREBRO P4%s · %d cara%s · %d persona%s · %d gato%s · mov %.0f%%",
             g_cfg.autoframe ? " AUTO" : "", faces, faces == 1 ? "" : "s", people, people == 1 ? "" : "s",
             cats, cats == 1 ? "" : "s", pipeline_motion_pct());

    time_t now = time(NULL);
    struct tm tm;
    localtime_r(&now, &tm);
    char hora[16] = "";
    if (tm.tm_year > 120) strftime(hora, sizeof(hora), "%H:%M", &tm);

    snprintf(right, sizeof(right), "%.0f fps · CPU %.0f/%.0f%% · %.0f°C%s%s%s%s · %s",
             stats_fps_out(), stats_cpu_pct(0), stats_cpu_pct(1), stats_temp(),
             wx[0] ? " · " : "", wx, money[0] ? " · " : "", money, hora);

    draw_text(16, (HUD_H - FONT_H) / 2, left, 255, 255, 255);
    int rw;
    hud_text_width(right, &rw);
    draw_text(s_w - rw - 16, (HUD_H - FONT_H) / 2, right, 180, 240, 255);

    esp_cache_msync(s_buf, s_cap, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
}
