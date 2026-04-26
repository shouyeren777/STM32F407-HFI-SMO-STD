## BLDC_PMSM_CONTROL

This directory is used for STM32F407 motor control code organization.

`User/MotorControl` files are migrated from the legacy motor-control source tree (`MOTOR/User/MotorControl`) without modification.

Do not directly modify algorithm source files under `User/MotorControl`.

Pin mapping baseline for F407 is defined in:

- `BSP/BLDC_PMSM_CONTROL/f407_bldc_pinmap.h`

F407-side platform bring-up (key/adc/pwm pin init) is placed in:

- `BLDC_PMSM_CONTROL/User/Platform/f407_bldc_platform.c`
