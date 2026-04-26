/**
 ****************************************************************************************************
 * @file        spi.c
 * @brief       SPI2 — PI0=CS, PI1=SCK, PI2=MISO, PI3=MOSI；DRV8301 使用 Mode1 + 16 位帧
 * @note        CS：默认空闲高、通信拉低（直连 nSCS）；反相接法在 spi.h 置 SPI2_CS_GPIO_ACTS_HIGH_DURING_XFER=1
 ****************************************************************************************************
 */

#include "./BSP/SPI/spi.h"

SPI_HandleTypeDef g_spi2_handler;

static void spi2_cs_inactive(void)
{
#if SPI2_CS_GPIO_ACTS_HIGH_DURING_XFER
    HAL_GPIO_WritePin(SPI2_CS_GPIO_PORT, SPI2_CS_GPIO_PIN, GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(SPI2_CS_GPIO_PORT, SPI2_CS_GPIO_PIN, GPIO_PIN_SET);
#endif
}

static void spi2_cs_active(void)
{
#if SPI2_CS_GPIO_ACTS_HIGH_DURING_XFER
    HAL_GPIO_WritePin(SPI2_CS_GPIO_PORT, SPI2_CS_GPIO_PIN, GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(SPI2_CS_GPIO_PORT, SPI2_CS_GPIO_PIN, GPIO_PIN_RESET);
#endif
}

void spi2_init(void)
{
    SPI2_SPI_CLK_ENABLE();

    g_spi2_handler.Instance = SPI2_SPI;
    g_spi2_handler.Init.Mode = SPI_MODE_MASTER;
    g_spi2_handler.Init.Direction = SPI_DIRECTION_2LINES;
    /* DRV8301：SPI Mode1（CPOL=0，CPHA=1，第二边沿采样） */
    g_spi2_handler.Init.DataSize = SPI_DATASIZE_16BIT;
    g_spi2_handler.Init.CLKPolarity = SPI_POLARITY_LOW;
    g_spi2_handler.Init.CLKPhase = SPI_PHASE_2EDGE;
    g_spi2_handler.Init.NSS = SPI_NSS_SOFT;
    /* 上电初值宜保守；DRV8301_Init 内会 spi2_set_speed(DRV8301_SPI_SPEED_INDEX) */
    g_spi2_handler.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    g_spi2_handler.Init.FirstBit = SPI_FIRSTBIT_MSB;
    g_spi2_handler.Init.TIMode = SPI_TIMODE_DISABLE;
    g_spi2_handler.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    g_spi2_handler.Init.CRCPolynomial = 7;
    HAL_SPI_Init(&g_spi2_handler);

    spi2_cs_inactive();
    __HAL_SPI_ENABLE(&g_spi2_handler);
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef GPIO_Initure;
    if (hspi->Instance == SPI2_SPI)
    {
        SPI2_CS_GPIO_CLK_ENABLE();
        SPI2_SCK_GPIO_CLK_ENABLE();
        SPI2_MISO_GPIO_CLK_ENABLE();
        SPI2_MOSI_GPIO_CLK_ENABLE();

        GPIO_Initure.Pin = SPI2_CS_GPIO_PIN;
        GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_Initure.Pull = GPIO_PULLUP;
        GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(SPI2_CS_GPIO_PORT, &GPIO_Initure);

        GPIO_Initure.Pin = SPI2_SCK_GPIO_PIN;
        GPIO_Initure.Mode = GPIO_MODE_AF_PP;
        GPIO_Initure.Pull = GPIO_PULLUP;
        GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_Initure.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(SPI2_SCK_GPIO_PORT, &GPIO_Initure);

        GPIO_Initure.Pin = SPI2_MISO_GPIO_PIN;
        HAL_GPIO_Init(SPI2_MISO_GPIO_PORT, &GPIO_Initure);

        GPIO_Initure.Pin = SPI2_MOSI_GPIO_PIN;
        HAL_GPIO_Init(SPI2_MOSI_GPIO_PORT, &GPIO_Initure);
    }
}

void spi2_set_speed(uint8_t speed)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(speed));
    __HAL_SPI_DISABLE(&g_spi2_handler);
    g_spi2_handler.Instance->CR1 &= 0XFFC7;
    g_spi2_handler.Instance->CR1 |= (uint32_t)speed << 3U;
    __HAL_SPI_ENABLE(&g_spi2_handler);
}

uint8_t spi2_read_write_byte(uint8_t txdata)
{
    uint8_t rxdata;
    /* 16 位模式下按字节访问不适用 DRV8301；保留接口供其它 8 位从机（需先改 DataSize） */
    HAL_SPI_TransmitReceive(&g_spi2_handler, &txdata, &rxdata, 1, 1000U);
    return rxdata;
}

uint16_t spi2_transfer16(uint16_t tx_word)
{
    uint16_t rx_word;
    spi2_cs_active();
    for (volatile int i = 0; i < 4; i++)
    {
        __NOP();
    }
    HAL_SPI_TransmitReceive(&g_spi2_handler, (uint8_t *)&tx_word, (uint8_t *)&rx_word, 1U, 1000U);
    spi2_cs_inactive();
    for (volatile int i = 0; i < 4; i++)
    {
        __NOP();
    }
    return rx_word;
}
