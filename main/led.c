// LED RGB WS2812 de la placa (GPIO51), manejado por el periférico RMT.
//   azul que respira = esperando al PC · verde = ve una cara · naranjo = ve un gato
//   blanco = trabajando · morado fijo = actualizando firmware · rojo = error
//   morado parpadeando = el puerto HUSB no está conectado a un PC (o Windows no lo reconoció)
#include <math.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "app.h"

#define LED_GPIO 51

static led_strip_handle_t s_led;
static volatile int s_state = LED_WAIT;

void led_state(int st) { s_state = st; }

static void led_task(void *arg)
{
    static const uint8_t col[][3] = {
        [LED_WAIT] = { 0, 40, 255 }, [LED_RUN] = { 160, 160, 160 }, [LED_FACE] = { 0, 255, 60 },
        [LED_CAT] = { 255, 110, 0 }, [LED_OTA] = { 170, 0, 255 }, [LED_ERR] = { 255, 0, 0 },
    };
    while (1) {
        int st = s_state;
        float t = esp_timer_get_time() / 1e6f;
        float k = (st == LED_WAIT) ? (0.15f + 0.85f * (0.5f + 0.5f * sinf(t * 2.5f))) : 0.35f;
        if (!usb_mounted() && st != LED_OTA && st != LED_ERR) {
            st = LED_OTA;
            k = ((int)(t * 2) & 1) ? 0.6f : 0.0f;
        }
        led_strip_set_pixel(s_led, 0, col[st][0] * k * 0.3f, col[st][1] * k * 0.3f, col[st][2] * k * 0.3f);
        led_strip_refresh(s_led);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void led_start(void)
{
    led_strip_config_t sc = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rc = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
    };
    if (led_strip_new_rmt_device(&sc, &rc, &s_led) == ESP_OK) {
        xTaskCreatePinnedToCore(led_task, "led", 3072, NULL, 4, NULL, 0);
    }
}
