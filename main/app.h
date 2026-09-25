// Cerebro P4 — declaraciones compartidas entre módulos.
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

#define P4_IP_STR    "192.168.7.1"
#define PORT_FRAMES  5000       // el PC empuja cuadros JPEG aquí
#define PORT_TUNNEL  5001       // el PC deja conexiones abiertas para darle internet al P4
#define PORT_WEB     80
#define PORT_STREAM  81

#define P4_ALIGN(x, a) (((x) + (a) - 1) & ~((a) - 1))

// ---------- Configuración que se cambia desde el dashboard ----------
typedef struct {
    bool  autoframe;     // la cámara sigue tu cara (recorta y hace zoom con el PPA)
    bool  mirror;        // espejo horizontal
    bool  hud;           // barra de texto
    bool  boxes;         // cuadros de detección
    bool  max_mode;      // todo encendido siempre (aunque nadie esté mirando)
    int   jpeg_q;        // calidad JPEG de salida 40..95
    float zoom_max;      // zoom máximo del auto-encuadre 1.0..3.0
    int   h264_kbps;     // bitrate del H.264
    bool  h264_on;       // grabar la repetición en H.264
    int   ai_level;      // 0 = IA apagada, 1 = caras, 2 = caras + personas, 3 = todo (también gatos)
} app_cfg_t;
extern app_cfg_t g_cfg;

// ---------- Detecciones de la IA ----------
typedef struct { int x0, y0, x1, y1; float score; int cls; } det_t;   // cls 0 = cara, 1 = gato, 2 = persona
#define MAX_DETS 12
typedef struct {
    det_t   d[MAX_DETS];
    int     n;
    int64_t t_us;
} det_list_t;

// ---------- Entrada de cuadros (ingest.c) ----------
typedef struct {
    uint8_t *buf;
    size_t   cap, len;
    uint32_t seq;
    int64_t  t_rx_us;
} jpeg_in_t;
void       ingest_start(void);
jpeg_in_t *ingest_get(TickType_t wait);
void       ingest_release(jpeg_in_t *f);
bool       ingest_pc_connected(void);
int        tunnel_open(const char *host, int port, int timeout_ms);   // socket al destino, pasando por el PC

// ---------- USB (usb.c) ----------
void     usb_start(void);
bool     usb_mounted(void);
bool     uvc_streaming(void);
int      uvc_format(void);                  // 1 = MJPEG, 2 = H.264
void     uvc_size(int *w, int *h);
bool     uvc_ready(void);                   // libre para mandar otro cuadro
bool     uvc_send(const uint8_t *buf, size_t len);
uint32_t uvc_frames_sent(void);

// ---------- Tubería de video (pipeline.c) ----------
void   pipeline_early_init(void);
void   pipeline_start(void);
size_t pipeline_copy_jpeg(uint8_t *dst, size_t cap, uint32_t *seq_io, TickType_t wait);
int    replay_stream(int (*emit)(void *ctx, const uint8_t *p, size_t n), void *ctx, int seconds);
void   pipeline_snapshot_request(void);
float  pipeline_motion_pct(void);            // % de la imagen en movimiento (vectores del H.264)

// ---------- IA (ai.cpp) ----------
#define AI_W 320                            // imagen reducida que comparten los 3 detectores
#define AI_H 180
void     ai_start(void);
uint8_t *ai_pool_begin_write(int *slot);    // NULL si nadie espera o no hay búfer libre
void     ai_pool_publish(int slot);
void     ai_get_dets(det_list_t *out);      // coordenadas normalizadas a 0..10000
void     ai_stats(float *face_fps, float *face_ms, float *cat_fps, float *cat_ms);
void     ai_stats_person(float *fps, float *ms);

// ---------- HUD (hud.c) ----------
void     hud_init(int w);
void     hud_render(void);
uint8_t *hud_buf(int *w, int *h);           // ARGB8888
void     hud_text_width(const char *s, int *px);

// ---------- Internet (inet.c) ----------
void inet_start(void);
void inet_status(char *weather, size_t wl, char *money, size_t ml, bool *ok, int *tls_ms, int *fetches);

// ---------- Estadísticas (stats.c) ----------
typedef enum { ENG_JDEC = 0, ENG_JENC, ENG_PPA, ENG_H264, ENG_USB, ENG_DMA2D, ENG_COUNT } eng_t;
void    stats_start(void);
void    stats_busy(eng_t e, int64_t t0_us, int64_t t1_us);
void    stats_frame_in(size_t bytes);
void    stats_frame_out(void);
void    stats_filler(void);
void    stats_net(size_t rx, size_t tx);
int     stats_json(char *buf, size_t cap);
float   stats_eng_pct(eng_t e);
float   stats_cpu_pct(int core);
float   stats_temp(void);
float   stats_fps_in(void);
float   stats_fps_out(void);
void    log_ring_init(void);
int     log_ring_copy(char *dst, int cap);

// ---------- LED RGB (led.c) ----------
enum { LED_WAIT = 0, LED_RUN, LED_FACE, LED_CAT, LED_OTA, LED_ERR };
void led_start(void);
void led_state(int st);

// ---------- Núcleo LP (lpcore.c) ----------
void lpcore_start(void);
void lpcore_stats(uint32_t *primes, uint32_t *last, uint32_t *loops);

// ---------- Web (web.c) ----------
void web_start(void);
void ota_mark_ok_if_pending(void);

#ifdef __cplusplus
}
#endif
