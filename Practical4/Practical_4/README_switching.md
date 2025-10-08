### Switching between PWM (TIM3 CCR via DMA) and DAC (PA4) builds

This project includes two alternative mains:
- PWM path (default): `Core/Src/main.c` → PB0 (TIM3_CH3 PWM) + RC + amplifier
- DAC path (alternate): `Core/Src/main_dac.c` → PA4 (DAC1_OUT1) + amplifier

Use one of the two options below to switch.

---

#### Option A: Quick rename swap (no Makefile edits)
1) To try the DAC build:
   - Rename current `main.c` → `main_pwm.c` (backup)
   - Rename `main_dac.c` → `main.c`
   - Build/flash as usual from `Practical_4/`
2) To go back to PWM:
   - Rename `main.c` (current DAC) → `main_dac.c`
   - Rename `main_pwm.c` → `main.c`

Notes:
- STM32CubeIDE users can do the same by excluding/including files from build or renaming inside the IDE.

---

#### Option B: Edit the Makefile to choose the entry file
The Makefile lists C sources under `C_SOURCES`. By default it includes:
- `Core/Src/main.c`

To build the DAC version instead, replace that entry with `Core/Src/main_dac.c`.

Example snippet (only show the relevant line):
- PWM build (default):
  - `Core/Src/main.c \`
- DAC build (alternate):
  - `Core/Src/main_dac.c \`

After editing, run from `Practical_4/`:
- `make clean && make`

Tip: You can keep two Makefile variants (or branches) and switch by file copy.

---

#### Hardware pins summary
- PWM build: Output on `PB0` (TIM3_CH3). Use an RC low‑pass then an amplifier.
- DAC build: Output on `PA4` (DAC1_CH1). You can drive your amplifier directly (an RC is optional for smoothing).
- Button: `PA0` (cycles Sine/Saw/Tri/Piano/Guitar/Drum)
- LCD: `PC14` (RS), `PC15` (E), `PB8` (D4), `PB9` (D5), `PA12` (D6), `PA15` (D7)
- Common ground is required between MCU and amplifier/speakers.

---

#### Troubleshooting
- If the LCD shows blocks, verify `GPIOC` clock is enabled and the wiring matches pins above.
- If you hear PWM buzz (PWM build), lower the RC corner frequency or use the DAC build.
- If nothing runs, confirm you are flashing the correct target (STM32F446), not a different board.
