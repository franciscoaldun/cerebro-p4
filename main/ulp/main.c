// Programa del núcleo LP (RISC-V de bajo consumo, 40 MHz): cuenta números primos sin parar.
// Trabaja en paralelo a los dos núcleos grandes; el dashboard lee sus contadores en vivo.
#include <stdint.h>
#include "ulp_lp_core_utils.h"

volatile uint32_t lp_primes = 1;       // ya cuenta el 2
volatile uint32_t lp_last_prime = 2;
volatile uint32_t lp_loops = 0;

int main(void)
{
    uint32_t n = 3;
    while (1) {
        uint32_t primo = 1;
        for (uint32_t d = 3; d * d <= n; d += 2) {
            if (n % d == 0) {
                primo = 0;
                break;
            }
        }
        if (primo) {
            lp_primes++;
            lp_last_prime = n;
        }
        n += 2;
        if (n > 0xFFF00000u) {
            n = 3;
        }
        lp_loops++;
    }
    return 0;
}
