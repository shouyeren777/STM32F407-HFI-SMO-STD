# HMI Migration Notes

- The following four directories are migrated from the legacy HMI source code:
  - `LcdControl`
  - `KeyControl`
  - `LedControl`
  - `UsartControl`
- Current stage is "as-is migration": only directory relocation is performed.
- Future F407 adaptation should be handled through the porting layer, including:
  - pin mapping
  - timer mapping
  - DMA/UART handle mapping
  - `main.h` dependency handling
