// Inteligencia artificial en la CPU del P4 (redes cuantizadas a 8 bits, instrucciones vectoriales PIE):
//   - caras    (ESPDet-Pico 224, respeta la proporción) -> núcleo 1, prioridad alta: guía el auto-encuadre
//   - personas (Pico 224)                               -> núcleo 1, cuando las caras están al día
//   - gatos    (ESPDet-Pico 224)                        -> núcleo 0, prioridad mínima: sólo usa CPU que sobra
// Los tres leen la MISMA imagen reducida (320x180): la tubería la escribe una vez en un búfer libre del
// "pozo" y cada detector toma la más nueva. Copiar imágenes con la CPU desde la PSRAM es caro en este
// chip, así que se copia lo mínimo.
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/idf_additions.h"
#include "human_face_detect.hpp"
#include "cat_detect.hpp"
#include "pedestrian_detect.hpp"
#include "app.h"

static const char *TAG = "ia";

#define N_AI   3
#define N_POOL 3
enum { AI_FACE = 0, AI_CAT = 1, AI_PERSON = 2 };

typedef struct {
    uint8_t *buf;
    int      readers;       // detectores leyéndolo ahora
    bool     writing;
    uint32_t seq;
} pool_t;

static pool_t            s_pool[N_POOL];
static int               s_newest = -1;
static uint32_t          s_seq;
static uint32_t          s_last_seen[N_AI];
static volatile bool     s_waiting[N_AI];
static SemaphoreHandle_t s_wake[N_AI];
static float             s_fps[N_AI], s_ms[N_AI];
static det_list_t        s_res[N_AI];
static portMUX_TYPE      s_mux = portMUX_INITIALIZER_UNLOCKED;

// ---------------- lado de la tubería ----------------
extern "C" uint8_t *ai_pool_begin_write(int *slot)
{
    bool want = false;
    for (int k = 0; k < N_AI; k++) want |= s_waiting[k];
    if (!want) return NULL;
    uint8_t *b = NULL;
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < N_POOL; i++) {
        if (i != s_newest && s_pool[i].readers == 0 && !s_pool[i].writing) {
            s_pool[i].writing = true;
            *slot = i;
            b = s_pool[i].buf;
            break;
        }
    }
    portEXIT_CRITICAL(&s_mux);
    return b;
}

extern "C" void ai_pool_publish(int slot)
{
    portENTER_CRITICAL(&s_mux);
    s_pool[slot].writing = false;
    s_pool[slot].seq = ++s_seq;
    s_newest = slot;
    portEXIT_CRITICAL(&s_mux);
    for (int k = 0; k < N_AI; k++) {
        if (s_waiting[k]) xSemaphoreGive(s_wake[k]);
    }
}

extern "C" void ai_get_dets(det_list_t *out)
{
    int64_t now = esp_timer_get_time();
    out->n = 0;
    out->t_us = 0;
    portENTER_CRITICAL(&s_mux);
    for (int k = 0; k < N_AI; k++) {
        if (now - s_res[k].t_us > 1200000) continue;          // resultados viejos no cuentan
        if (s_res[k].t_us > out->t_us) out->t_us = s_res[k].t_us;
        for (int i = 0; i < s_res[k].n && out->n < MAX_DETS; i++) out->d[out->n++] = s_res[k].d[i];
    }
    portEXIT_CRITICAL(&s_mux);
}

extern "C" void ai_stats(float *face_fps, float *face_ms, float *cat_fps, float *cat_ms)
{
    *face_fps = s_fps[AI_FACE];
    *face_ms = s_ms[AI_FACE];
    *cat_fps = s_fps[AI_CAT];
    *cat_ms = s_ms[AI_CAT];
}

extern "C" void ai_stats_person(float *fps, float *ms)
{
    *fps = s_fps[AI_PERSON];
    *ms = s_ms[AI_PERSON];
}

// ---------------- detectores ----------------
static int take_newest(int which)
{
    int slot = -1;
    portENTER_CRITICAL(&s_mux);
    if (s_newest >= 0 && s_pool[s_newest].seq != s_last_seen[which]) {
        slot = s_newest;
        s_pool[slot].readers++;
        s_last_seen[which] = s_pool[slot].seq;
    }
    portEXIT_CRITICAL(&s_mux);
    return slot;
}

static void ai_task(void *arg)
{
    int which = (int)(intptr_t)arg;
    dl::detect::Detect *det = nullptr;
    int cls = 0;
    if (which == AI_FACE) {
        det = new HumanFaceDetect();
        cls = 0;
    } else if (which == AI_CAT) {
        det = new CatDetect();
        cls = 1;
    } else {
        det = new PedestrianDetect();
        cls = 2;
    }
    ESP_LOGI(TAG, "detector %d listo en el núcleo %d", which, xPortGetCoreID());
    int64_t win_t0 = esp_timer_get_time();
    int win_n = 0;
    float ms_acc = 0;
    while (true) {
        int slot;
        int need = which == AI_FACE ? 1 : which == AI_PERSON ? 2 : 3;   // nivel de IA que activa a este detector
        if (g_cfg.ai_level < need) {
            portENTER_CRITICAL(&s_mux);
            s_res[which].n = 0;
            portEXIT_CRITICAL(&s_mux);
            s_fps[which] = 0;
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        while ((slot = take_newest(which)) < 0) {
            s_waiting[which] = true;
            xSemaphoreTake(s_wake[which], pdMS_TO_TICKS(200));
        }
        s_waiting[which] = false;
        dl::image::img_t img;
        img.data = s_pool[slot].buf;
        img.width = AI_W;
        img.height = AI_H;
        img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_BGR888;
        int64_t t0 = esp_timer_get_time();
        std::list<dl::detect::result_t> &res = det->run(img);
        int64_t t1 = esp_timer_get_time();
        portENTER_CRITICAL(&s_mux);
        s_pool[slot].readers--;
        portEXIT_CRITICAL(&s_mux);

        det_list_t dl;
        dl.n = 0;
        dl.t_us = t1;
        for (auto &r : res) {
            if (dl.n >= MAX_DETS / 2) break;
            det_t &d = dl.d[dl.n++];
            d.x0 = r.box[0] * 10000 / AI_W;
            d.y0 = r.box[1] * 10000 / AI_H;
            d.x1 = r.box[2] * 10000 / AI_W;
            d.y1 = r.box[3] * 10000 / AI_H;
            d.score = r.score;
            d.cls = cls;
        }
        portENTER_CRITICAL(&s_mux);
        s_res[which] = dl;
        portEXIT_CRITICAL(&s_mux);

        win_n++;
        ms_acc += (t1 - t0) / 1000.0f;
        if (t1 - win_t0 > 1000000) {
            s_fps[which] = win_n * 1e6f / (t1 - win_t0);
            s_ms[which] = ms_acc / win_n;
            win_t0 = t1;
            win_n = 0;
            ms_acc = 0;
        }
    }
}

extern "C" void ai_start(void)
{
    for (int i = 0; i < N_POOL; i++) {
        s_pool[i].buf = (uint8_t *)heap_caps_aligned_calloc(64, 1, P4_ALIGN(AI_W * AI_H * 3, 64), MALLOC_CAP_SPIRAM);
    }
    for (int k = 0; k < N_AI; k++) s_wake[k] = xSemaphoreCreateBinary();
    // pilas en PSRAM: la RAM interna queda para USB, red y H.264
    xTaskCreatePinnedToCoreWithCaps(ai_task, "ia_caras", 16384, (void *)AI_FACE, 5, NULL, 1, MALLOC_CAP_SPIRAM);
    xTaskCreatePinnedToCoreWithCaps(ai_task, "ia_personas", 16384, (void *)AI_PERSON, 4, NULL, 1, MALLOC_CAP_SPIRAM);
    xTaskCreatePinnedToCoreWithCaps(ai_task, "ia_gatos", 16384, (void *)AI_CAT, 1, NULL, 0, MALLOC_CAP_SPIRAM);
}
