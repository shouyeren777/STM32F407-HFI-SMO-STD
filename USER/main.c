#include "main.h"
#include "../BLDC_PMSM_CONTROL/User/GlobalControl/global_control.h"

int main(void)
{
    HAL_Init();

    /* HSE = 25 MHz external crystal, SYSCLK = 168 MHz. */
    sys_stm32_clock_init(336, 25, 2, 7);
    delay_init(168);

    /* F407 register-level bring-up aligned with migrated control flow. */
    bldc_platform_init();

    Global_Init();
    for (;;) {
        Global_Loop();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}
