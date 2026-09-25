// Cerebro P4 — WT9932P4-TINY (ESP32-P4 v1.0, 32 MB PSRAM).
// Recibe la cámara del PC por USB, corre IA, sigue tu cara y devuelve una webcam + dashboard.
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_heap_caps.h"
#include "esp_chip_info.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_system.h"
#include <stdlib.h>

app_cfg_t g_cfg = {
    .autoframe = true,
    .mirror = false,
    .hud = true,
    .boxes = true,
    .max_mode = true,           // pedido: todo el silicio al máximo (perfil "Máximo silicio")
    .jpeg_q = 82,
    .zoom_max = 2.5f,
    .h264_kbps = 4000,
    .h264_on = true,
    .ai_level = 3,
};

// Consola por el puerto FUSB (sirve para probar sin el PC):
//   m = modo máximo on/off · s = estadísticas JSON · r = reiniciar
static void console_task(void *arg)
{
    usb_serial_jtag_driver_config_t cfg = { .tx_buffer_size = 4096, .rx_buffer_size = 256 };
    if (usb_serial_jtag_driver_install(&cfg) != ESP_OK) {
        vTaskDelete(NULL);
    }
    usb_serial_jtag_vfs_use_driver();
    char *buf = malloc(4096);
    while (1) {
        uint8_t c;
        if (usb_serial_jtag_read_bytes(&c, 1, portMAX_DELAY) != 1) continue;
        if (c == 'm') {
            g_cfg.max_mode = !g_cfg.max_mode;
            ESP_LOGW("consola", "modo máximo %s", g_cfg.max_mode ? "ENCENDIDO" : "apagado");
        } else if (c == 's') {
            stats_json(buf, 4096);
            printf("STATS %s\n", buf);
        } else if (c == 'r') {
            esp_restart();
        }
    }
}

static void mem_log(const char *paso)
{
    ESP_LOGI("mem", "%-10s RAM interna libre %3u KB (bloque mayor %3u KB) | PSRAM libre %5u KB", paso,
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
             (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
}

void app_main(void)
{
    mem_log("inicio");
    log_ring_init();
    pipeline_early_init();      // primero: el H.264 necesita RAM interna contigua
    mem_log("h264");
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_chip_info_t ci;
    esp_chip_info(&ci);
    ESP_LOGI("main", "Cerebro P4: chip rev v%d.%d, %d núcleos, PSRAM %u KB libres",
             ci.revision / 100, ci.revision % 100, ci.cores,
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));

    led_start();
    mem_log("led");
    stats_start();
    mem_log("stats");
    lpcore_start();
    mem_log("lpcore");
    usb_start();
    mem_log("usb");
    ingest_start();
    mem_log("ingest");
    ai_start();
    mem_log("ai");
    pipeline_start();
    mem_log("pipeline");
    web_start();
    mem_log("web");
    inet_start();
    mem_log("inet");
    xTaskCreatePinnedToCore(console_task, "consola", 6144, NULL, 5, NULL, 0);
}
