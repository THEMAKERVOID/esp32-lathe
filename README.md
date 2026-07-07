# TheMakerVoid ELS — ESP32-S3 Electronic Lead Screw

An open-source **Electronic Lead Screw (ELS)** controller for manual lathes, built around the ESP32-S3 and a 320×240 ILI9341 TFT touchscreen. Synchronizes a servo/stepper motor with spindle rotation to automate feed and thread cutting operations.

---

## Features

- **4 operating modes** — Feed, Thread, Divider, and Configuration
- **Metric & Imperial (UTS) threads** — 16 metric pitches (0.20–4.00 mm) and 21 imperial pitches (8–56 TPI), selectable from the Thread Unit Switch screen
- **5 feed rates** — 0.07, 0.10, 0.13, 0.18, 0.25 mm/rev
- **Angular divider** — real-time angle readout from encoder
- **Dual input** — XPT2046 resistive touchscreen (debounced) and 4 physical buttons
- **Dark mode UI** — Figma-designed dark interface with Orbitron bitmap fonts
- **EEPROM persistence** — motor direction, lead screw pitch, and dark mode saved across power cycles
- **Dual-core architecture** — UI on Core0, servo synchronization on Core1 (FreeRTOS)
- **Bresenham pulse algorithm** — precise stepper pulse generation without floating-point drift

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | ESP32-S3 DevKit C-1 |
| Display | ILI9341 TFT 320×240, landscape |
| Encoder | Quadrature rotary encoder, 1440 pulses/rev |
| Servo/Stepper driver | Lichuan A6 (PUL/DIR interface) |
| Touchscreen | XPT2046 resistive controller (on the ILI9341 module) |
| Input | XPT2046 touchscreen + 4 physical buttons: UP, DOWN, SELECT, BACK |

### Pin Map

| Function | GPIO |
|---|---|
| TFT CS / DC / RST | 10 / 11 / 12 |
| TFT SCK / MOSI / MISO | 15 / 13 / 14 |
| Touch CS / IRQ | 16 / 255 (IRQ unused) |
| Encoder A / B | 20 / 21 |
| Stepper PUL / DIR | 8 / 9 |
| BTN UP / DOWN / SELECT / BACK | 4 / 5 / 6 / 7 |
| Brake (reserved) | 3 |
| Encoder Z index (reserved) | 2 |

---

## UI Screens

| Screen | Description |
|---|---|
| Splash | Logo + boot animation |
| Main Menu | Mode selection + live RPM |
| Feed Select | Feed rate selection grid |
| Feed Mode | Active feed display + live RPM |
| Thread Unit Switch | Select Metric (mm) or Imperial (TPI) |
| Thread Select | Paginated pitch grid |
| Thread Mode | Active thread display + live RPM |
| Divider | Real-time angular position |
| Menu Overlay | Navigation hub |
| Config — Direction | CW / CCW motor direction |
| Config — Lead Screw Pitch | 1–10 mm digit selector |
| Interface | Dark mode toggle |
| Info | Firmware version and credits |

---

## Software Setup

### Requirements

- [Arduino IDE 2.x](https://www.arduino.cc/en/software)
- ESP32-S3 board package (via Boards Manager: `esp32` by Espressif)
- Libraries (via Library Manager):
  - Adafruit GFX Library
  - Adafruit ILI9341
  - XPT2046_Touchscreen
  - EEPROM (included with ESP32 core)

### Build & Flash

1. Clone or download this repository
2. Open `esp32-lathe/esp32-lathe.ino` in Arduino IDE 2.x
3. Select board: **ESP32S3 Dev Module**
4. Flash partition scheme: **Huge APP (3MB No OTA/1MB SPIFFS)** (recommended for PROGMEM bitmaps)
5. Upload

> **Note:** VS Code IntelliSense may show false errors for ESP32 headers. Arduino IDE 2.x compiles correctly.

---

## Project Structure

```
esp32-lathe/
├── esp32-lathe.ino       # Main sketch
├── bitmaps/              # PROGMEM RGB565 bitmap headers
│   ├── logo_splash.h
│   ├── bg_main.h
│   ├── btn_menu.h
│   ├── btn_back.h
│   ├── btn_ok.h
│   ├── btn_reload.h
│   ├── btn_stepper.h
│   ├── solid.h
│   └── btn_slider.h
├── Fonts/                # Orbitron & Montserrat bitmap font headers
│   ├── Orbitron_Medium8pt7b.h
│   ├── Orbitron_Medium12pt7b.h
│   ├── Orbitron_Medium18pt7b.h
│   ├── Orbitron_Medium24pt7b.h
│   └── Montserrat_Light6pt7b.h
└── Resources/            # User manuals and component list (see Documentation)
    ├── User_Manual.doc        # User manual (English)
    ├── Manual_Usuario.doc     # User manual (Spanish)
    └── ELS_Componentes.docx   # Bill of materials / component list
```

---

## How It Works

The encoder mounted on the lathe spindle generates quadrature pulses (1440 p/rev). Core1 runs a high-priority FreeRTOS task that reads the encoder count and computes stepper pulses using a **Bresenham integer accumulator**:

```
accumulator += |encoderDelta| × (desiredPitch × 100)
pulsesToSend  = accumulator / (leadScrewPitch × 100)
accumulator  %= (leadScrewPitch × 100)
```

This produces exact pulse ratios without floating-point error accumulation, enabling accurate thread cutting and feed synchronization.

---

## Documentation

Full user manuals and the component list are available in the [`esp32-lathe/Resources/`](esp32-lathe/Resources/) folder:

| Document | Language | Description |
|---|---|---|
| [User_Manual.doc](esp32-lathe/Resources/User_Manual.doc) | English | Complete user manual — operation, modes, and configuration |
| [Manual_Usuario.doc](esp32-lathe/Resources/Manual_Usuario.doc) | Español | Manual de usuario completo — funcionamiento, modos y configuración |
| [ELS_Componentes.docx](esp32-lathe/Resources/ELS_Componentes.docx) | Español | Bill of materials / lista de componentes |

---

## License

This project is open-source. See [LICENSE](LICENSE) for details.

---

## Credits

**TheMakerVoid** — [www.themakervoid.com](https://www.themakervoid.com)

Contributions, issues, and pull requests are welcome.
