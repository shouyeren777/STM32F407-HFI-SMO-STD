# STM32F407 Porting Rules

These rules define the target code organization for the STM32F407 project.

## Directory responsibilities

- `BLDC_PMSM_CONTROL/`
  - All motor-run related code for the F407 target lives here.
  - `User/MotorControl` is migrated from `MOTOR/User/MotorControl` and must remain unchanged.

- `UI_and_CONTROL/`
  - Human-machine interaction code only (screen, touch, keys, UI tasks, etc).

- `BSP/`
  - F407 platform and library source code (chip support, board support, HAL-related sources).

## Migration constraints (Step 1)

- Motor algorithm files from the legacy source project must keep original file structure.
- Motor algorithm file content must not be changed directly.
- F407 pin mapping must follow `BSP/BLDC_PMSM_CONTROL/f407_bldc_pinmap.h`.
