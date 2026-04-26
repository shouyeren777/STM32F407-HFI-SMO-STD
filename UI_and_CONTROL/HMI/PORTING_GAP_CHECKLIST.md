# HMI -> F407 Remaining Porting Items

This checklist tracks what is still required after the as-is migration of legacy HMI sources.

## 1) Build/Include Port Layer

- Current HMI headers include `main.h` directly.
- F407 project needs a dedicated port header (for example `f407_hmi_port.h`) to provide:
  - basic typedef aliases used by HMI (`u8`, `u16`, `u32`, `s8`)
  - HAL handles used by HMI (`hspi3`, `huart1`, `hdma_usart1_tx`)
  - shared APIs used in UI flow (`HAL_Delay`, `HAL_GPIO_*`, `HAL_SPI_*`, `HAL_UART_*`)

## 2) Key Mapping Adaptation

- Current `KeyControl/key_drv.h` still uses legacy key pins:
  - `KEY1 -> PC9`
  - `KEY2 -> PB12`
- Must be remapped to F407 pin plan:
  - `KEY0 -> PE2`
  - `KEY1 -> PE3`
  - `KEY2 -> PE4`
- Decide behavior mapping (2-key logic compatibility vs 3-key native logic).

## 3) LCD GPIO/SPI Mapping Validation

- Current `LcdControl/lcd_drv.h` uses:
  - `RES -> PC11`
  - `DC  -> PD2`
  - `CS  -> PA15`
  - SPI handle `hspi3`
- Confirm these pins and SPI instance match F407 board wiring.

## 4) Task Scheduler Hooking

- HMI task model depends on periodic timers:
  - `LedTaskTim`, `LcdTaskTim`, `KeyTaskTim`, `UsartTaskTim`
- Need F407 scheduler/timer callback integration equivalent to source `TIM2` task cadence.

## 5) UART DMA Telemetry Path

- `UsartControl/usart_task.c` requires:
  - `huart1`
  - `hdma_usart1_tx`
  - DMA counter check + DMA TX restart flow
- Verify F407 USART/DMA initialization and channel mapping before enabling this task.

## 6) Motor Data Dependency

- HMI uses `motor_system.h` and global `MC` data model.
- Ensure include path points to:
  - `BLDC_PMSM_CONTROL/User/MotorControl`
- Ensure motor runtime is initialized before HMI dynamic pages are enabled.

## 7) Integration Entry Points

- Add explicit UI entry functions on F407 side (example):
  - `hmi_init()`
  - `hmi_task_100us()`
  - `hmi_loop()`
- Keep UI module location under `UI_and_CONTROL` only.
