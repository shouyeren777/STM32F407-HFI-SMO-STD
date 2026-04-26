#include "f407_bldc_platform.h"

#include "../../../BSP/BLDC_PMSM_CONTROL/f407_bldc_pinmap.h"

ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

TIM_HandleTypeDef htim1; /* Mapped to TIM8 */
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

static void bldc_gpio_clock_enable(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();
}

static void bldc_dma_clock_enable(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
}

void bldc_key_input_init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    gpio_init.Pin = KEY0_GPIO_PIN | KEY1_GPIO_PIN | KEY2_GPIO_PIN;
    gpio_init.Mode = GPIO_MODE_INPUT;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(KEY0_GPIO_PORT, &gpio_init);
}

static void bldc_led_output_init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN | LED2_GPIO_PIN, GPIO_PIN_RESET);
    gpio_init.Pin = LED1_GPIO_PIN | LED2_GPIO_PIN;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED1_GPIO_PORT, &gpio_init);
}

void bldc_adc_bemf_amp_input_init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    gpio_init.Mode = GPIO_MODE_ANALOG;
    gpio_init.Pull = GPIO_NOPULL;

    gpio_init.Pin = BEMFW_GPIO_PIN | BEMFV_GPIO_PIN | BEMFU_GPIO_PIN;
    HAL_GPIO_Init(BEMFU_GPIO_PORT, &gpio_init);

    gpio_init.Pin = AMPW_GPIO_PIN | AMPV_GPIO_PIN;
    HAL_GPIO_Init(GPIOA, &gpio_init);

    gpio_init.Pin = AMPU_GPIO_PIN;
    HAL_GPIO_Init(AMPU_GPIO_PORT, &gpio_init);
}

void bldc_pwm_gpio_init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.Alternate = GPIO_AF3_TIM8;

    gpio_init.Pin = PWM_UH_GPIO_PIN | PWM_VH_GPIO_PIN | PWM_WH_GPIO_PIN;
    HAL_GPIO_Init(PWM_UH_GPIO_PORT, &gpio_init);

    gpio_init.Pin = PWM_UL_GPIO_PIN | PWM_VL_GPIO_PIN | PWM_WL_GPIO_PIN;
    HAL_GPIO_Init(PWM_UL_GPIO_PORT, &gpio_init);
}

static void bldc_spi2_register_init(void)
{
    hspi1.Instance = SPI2;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    HAL_SPI_Init(&hspi1);
}

static void bldc_uart1_register_init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();

    gpio_init.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &gpio_init);

    huart1.Instance = USART3;
    huart1.Init.BaudRate = 2000000;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    HAL_NVIC_SetPriority(USART3_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

static void bldc_uart1_dma_register_init(void)
{
    hdma_usart1_tx.Instance = DMA1_Stream3;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_usart1_tx);
    __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

static void bldc_adc2_register_init(void)
{
    ADC_ChannelConfTypeDef s_config = {0};
    ADC_InjectionConfTypeDef s_injected_config = {0};

    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc2.Init.Resolution = ADC_RESOLUTION_12B;
    hadc2.Init.ScanConvMode = ENABLE;
    hadc2.Init.ContinuousConvMode = ENABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.NbrOfDiscConversion = 0;
    hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion = 3;
    hadc2.Init.DMAContinuousRequests = ENABLE;
    hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    HAL_ADC_Init(&hadc2);

    /*
     * F407 board pinout differs from the legacy source platform.
     * Here we map regular conversion to three available analog inputs.
     */
    s_config.Channel = ADC_CHANNEL_3;
    s_config.Rank = 1;
    s_config.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    HAL_ADC_ConfigChannel(&hadc2, &s_config);

    s_config.Channel = ADC_CHANNEL_6;
    s_config.Rank = 2;
    HAL_ADC_ConfigChannel(&hadc2, &s_config);

    s_config.Channel = ADC_CHANNEL_8;
    s_config.Rank = 3;
    HAL_ADC_ConfigChannel(&hadc2, &s_config);

    s_injected_config.InjectedChannel = ADC_CHANNEL_8;
    s_injected_config.InjectedRank = 1;
    s_injected_config.InjectedNbrOfConversion = 2;
    s_injected_config.InjectedSamplingTime = ADC_SAMPLETIME_84CYCLES;
    s_injected_config.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
    s_injected_config.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
    s_injected_config.AutoInjectedConv = DISABLE;
    s_injected_config.InjectedDiscontinuousConvMode = DISABLE;
    s_injected_config.InjectedOffset = 0;
    HAL_ADCEx_InjectedConfigChannel(&hadc2, &s_injected_config);

    s_injected_config.InjectedChannel = ADC_CHANNEL_3;
    s_injected_config.InjectedRank = 2;
    HAL_ADCEx_InjectedConfigChannel(&hadc2, &s_injected_config);
}

static void bldc_adc3_register_init(void)
{
    ADC_ChannelConfTypeDef s_config = {0};

    hadc3.Instance = ADC3;
    hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc3.Init.Resolution = ADC_RESOLUTION_12B;
    hadc3.Init.ScanConvMode = ENABLE;
    hadc3.Init.ContinuousConvMode = ENABLE;
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.NbrOfDiscConversion = 0;
    hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc3.Init.NbrOfConversion = 3;
    hadc3.Init.DMAContinuousRequests = DISABLE;
    hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    HAL_ADC_Init(&hadc3);

    /* BEMFU/PF9 -> ADC3_IN7 */
    s_config.Channel = ADC_CHANNEL_7;
    s_config.Rank = 1;
    s_config.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    HAL_ADC_ConfigChannel(&hadc3, &s_config);

    /* BEMFV/PF8 -> ADC3_IN6 */
    s_config.Channel = ADC_CHANNEL_6;
    s_config.Rank = 2;
    HAL_ADC_ConfigChannel(&hadc3, &s_config);

    /* BEMFW/PF7 -> ADC3_IN5 */
    s_config.Channel = ADC_CHANNEL_5;
    s_config.Rank = 3;
    HAL_ADC_ConfigChannel(&hadc3, &s_config);
}

static void bldc_tim8_pwm_register_init(void)
{
    TIM_ClockConfigTypeDef s_clock_source_config = {0};
    TIM_MasterConfigTypeDef s_master_config = {0};
    TIM_OC_InitTypeDef s_config_oc = {0};
    TIM_BreakDeadTimeConfigTypeDef s_break_dead_time_config = {0};

    htim1.Instance = TIM8;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    htim1.Init.Period = 4250;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 1;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim1);

    s_clock_source_config.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim1, &s_clock_source_config);
    HAL_TIM_PWM_Init(&htim1);

    s_master_config.MasterOutputTrigger = TIM_TRGO_RESET;
    s_master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim1, &s_master_config);

    s_config_oc.OCMode = TIM_OCMODE_PWM2;
    s_config_oc.Pulse = 0;
    s_config_oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    s_config_oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    s_config_oc.OCFastMode = TIM_OCFAST_DISABLE;
    s_config_oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    s_config_oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim1, &s_config_oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &s_config_oc, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &s_config_oc, TIM_CHANNEL_3);

    s_break_dead_time_config.OffStateRunMode = TIM_OSSR_DISABLE;
    s_break_dead_time_config.OffStateIDLEMode = TIM_OSSI_DISABLE;
    s_break_dead_time_config.LockLevel = TIM_LOCKLEVEL_OFF;
    s_break_dead_time_config.DeadTime = 0;
    s_break_dead_time_config.BreakState = TIM_BREAK_DISABLE;
    s_break_dead_time_config.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    s_break_dead_time_config.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &s_break_dead_time_config);

    HAL_TIM_MspPostInit(&htim1);
}

static void bldc_tim2_task_register_init(void)
{
    TIM_ClockConfigTypeDef s_clock_source_config = {0};
    TIM_MasterConfigTypeDef s_master_config = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 83;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 99;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim2);

    s_clock_source_config.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim2, &s_clock_source_config);

    s_master_config.MasterOutputTrigger = TIM_TRGO_RESET;
    s_master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &s_master_config);
}

static void bldc_tim6_register_init(void)
{
    TIM_MasterConfigTypeDef s_master_config = {0};

    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 167;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 65535;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim6);

    s_master_config.MasterOutputTrigger = TIM_TRGO_RESET;
    s_master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim6, &s_master_config);
}

void bldc_registers_init(void)
{
    __HAL_RCC_ADC2_CLK_ENABLE();
    __HAL_RCC_ADC3_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM6_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();

    bldc_adc2_register_init();
    bldc_adc3_register_init();
    bldc_tim8_pwm_register_init();
    bldc_tim2_task_register_init();
    bldc_tim6_register_init();
    bldc_spi2_register_init();
    bldc_uart1_register_init();
    bldc_uart1_dma_register_init();
}

void bldc_platform_init(void)
{
    bldc_gpio_clock_enable();
    bldc_dma_clock_enable();

    bldc_key_input_init();
    bldc_led_output_init();
    bldc_adc_bemf_amp_input_init();
    bldc_pwm_gpio_init();
    bldc_registers_init();
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    GPIO_InitTypeDef gpio_init = {0};

    if (hadc->Instance == ADC2) {
        __HAL_RCC_ADC2_CLK_ENABLE();

        gpio_init.Pin = GPIO_PIN_3 | GPIO_PIN_6;
        gpio_init.Mode = GPIO_MODE_ANALOG;
        gpio_init.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &gpio_init);

        gpio_init.Pin = GPIO_PIN_0;
        HAL_GPIO_Init(GPIOB, &gpio_init);

        hdma_adc2.Instance = DMA2_Stream2;
        hdma_adc2.Init.Channel = DMA_CHANNEL_1;
        hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
        hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
        hdma_adc2.Init.Mode = DMA_CIRCULAR;
        hdma_adc2.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_adc2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_adc2);
        __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc2);

        HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
        HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(ADC_IRQn);
    } else if (hadc->Instance == ADC3) {
        __HAL_RCC_ADC3_CLK_ENABLE();
        __HAL_RCC_GPIOF_CLK_ENABLE();

        gpio_init.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
        gpio_init.Mode = GPIO_MODE_ANALOG;
        gpio_init.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOF, &gpio_init);
    }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base)
{
    if (htim_base->Instance == TIM8) {
        __HAL_RCC_TIM8_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
    } else if (htim_base->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    } else if (htim_base->Instance == TIM6) {
        __HAL_RCC_TIM6_CLK_ENABLE();
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gpio_init = {0};

    if (htim->Instance != TIM8) {
        return;
    }

    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.Alternate = GPIO_AF3_TIM8;

    gpio_init.Pin = PWM_UH_GPIO_PIN | PWM_VH_GPIO_PIN | PWM_WH_GPIO_PIN;
    HAL_GPIO_Init(PWM_UH_GPIO_PORT, &gpio_init);

    gpio_init.Pin = PWM_UL_GPIO_PIN | PWM_VL_GPIO_PIN | PWM_WL_GPIO_PIN;
    HAL_GPIO_Init(PWM_UL_GPIO_PORT, &gpio_init);
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef gpio_init = {0};

    if (hspi->Instance == SPI2) {
        __HAL_RCC_SPI2_CLK_ENABLE();
        __HAL_RCC_GPIOI_CLK_ENABLE();

        /* PI1=SCK, PI2=MISO, PI3=MOSI */
        gpio_init.Pin = SPI_CLK_GPIO_PIN | SPI_MISO_GPIO_PIN | SPI_MOSI_GPIO_PIN;
        gpio_init.Mode = GPIO_MODE_AF_PP;
        gpio_init.Pull = GPIO_NOPULL;
        gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio_init.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(SPI_CLK_GPIO_PORT, &gpio_init);

        HAL_GPIO_WritePin(SPI_CS_GPIO_PORT, SPI_CS_GPIO_PIN, GPIO_PIN_SET);
        gpio_init.Pin = SPI_CS_GPIO_PIN;
        gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
        gpio_init.Pull = GPIO_PULLUP;
        gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(SPI_CS_GPIO_PORT, &gpio_init);
    }
}
