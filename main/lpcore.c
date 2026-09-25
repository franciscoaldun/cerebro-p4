// Arranca el núcleo LP del P4 con su propio programa (ulp/main.c) y lee sus contadores.
#include "esp_log.h"
#include "ulp_lp_core.h"
#include "lp_core_main.h"
#include "app.h"

extern const uint8_t lp_core_main_bin_start[] asm("_binary_lp_core_main_bin_start");
extern const uint8_t lp_core_main_bin_end[]   asm("_binary_lp_core_main_bin_end");

static bool s_ok;

void lpcore_start(void)
{
    ulp_lp_core_cfg_t cfg = {
        .wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_HP_CPU,
    };
    if (ulp_lp_core_load_binary(lp_core_main_bin_start, lp_core_main_bin_end - lp_core_main_bin_start) == ESP_OK &&
        ulp_lp_core_run(&cfg) == ESP_OK) {
        s_ok = true;
        ESP_LOGI("lp", "núcleo LP corriendo (buscador de primos)");
    } else {
        ESP_LOGE("lp", "no se pudo arrancar el núcleo LP");
    }
}

void lpcore_stats(uint32_t *primes, uint32_t *last, uint32_t *loops)
{
    if (!s_ok) {
        *primes = *last = *loops = 0;
        return;
    }
    *primes = ulp_lp_primes;
    *last = ulp_lp_last_prime;
    *loops = ulp_lp_loops;
}
