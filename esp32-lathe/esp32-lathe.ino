#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <rom/ets_sys.h>   // ets_delay_us() — delay without disabling interrupts (Core1)
#include <XPT2046_Touchscreen.h>
#include <EEPROM.h>
#include <math.h>
#include "bitmaps/logo_splash.h"
#include "bitmaps/bg_main.h"
#include "bitmaps/btn_menu.h"
#include "bitmaps/btn_back.h"
#include "bitmaps/btn_ok.h"
#include "bitmaps/btn_reload.h"
#include "bitmaps/btn_stepper.h"
#include "bitmaps/solid.h"
#include "bitmaps/btn_slider.h"
#include "Fonts/Orbitron_Medium8pt7b.h"
#include "Fonts/Orbitron_Medium12pt7b.h"
#include "Fonts/Orbitron_Medium18pt7b.h"
#include "Fonts/Orbitron_Medium24pt7b.h"
#include "Fonts/Montserrat_Light6pt7b.h"

#define TFT_CS 10
#define TFT_DC 11
#define TFT_RST 12
#define TFT_SCK 15
#define TFT_MOSI 13
#define TFT_MISO 14
#define TOUCH_CS 16
#define TOUCH_IRQ 255
#define BTN_UP 4
#define BTN_DOWN 5
#define BTN_SELECT 6
#define BTN_BACK 7
#define ENC_A 20
#define ENC_B 21
#define PUL_PIN 8
#define DIR_PIN 9
#define BRAKE_PIN 3  // GPIO for electromagnetic brake control
#define ENC_Z 2      // GPIO for encoder Z (index) signal - reserved

XPT2046_Touchscreen touch(TOUCH_CS, TOUCH_IRQ);

#define EEPROM_MAGIC_ADDR 0
#define EEPROM_DIRECTION_ADDR 1
#define EEPROM_LEADSCREW_ADDR 2
#define EEPROM_DARKMODE_ADDR 3
#define EEPROM_MAGIC_VALUE 0xAC  // Bumped to force re-init after adding darkMode

constexpr int QUAD = 1440;
constexpr float LEAD = 3.0f;
constexpr uint16_t UPDATE_RPM = 200;
constexpr uint16_t UPDATE_ANGLE = 50;
constexpr uint16_t DEBOUNCE_DELAY = 200;
constexpr int TS_MINX = 395;   // Touch calibration — adjust for your panel
constexpr int TS_MAXX = 3846;
constexpr int TS_MINY = 412;
constexpr int TS_MAXY = 3739;
constexpr uint16_t TOUCH_DEBOUNCE = 250;  // ms between touch events

const float avances[] = {0.07f, 0.10f, 0.13f, 0.18f, 0.25f};
const int NUM_AV = 5;
// Metric thread pitches (mm)
const float roscas[] = {0.20f, 0.30f, 0.40f, 0.50f, 0.60f, 0.70f, 0.80f, 1.00f,
                         1.25f, 1.50f, 1.75f, 2.00f, 2.50f, 3.00f, 3.50f, 4.00f};
const int NUM_ROS = 16;

// Imperial thread pitches — TPI values and their mm equivalents
const float tpi_vals[]        = {8, 9, 9.5f, 10, 11, 12, 14, 16, 18, 19,
                                  20, 22, 24, 28, 32, 36, 38, 40, 44, 48, 56};
const float roscas_imperial[] = {25.4f/8,  25.4f/9,  25.4f/9.5f, 25.4f/10, 25.4f/11,
                                  25.4f/12, 25.4f/14, 25.4f/16,   25.4f/18, 25.4f/19,
                                  25.4f/20, 25.4f/22, 25.4f/24,   25.4f/28, 25.4f/32,
                                  25.4f/36, 25.4f/38, 25.4f/40,   25.4f/44, 25.4f/48,
                                  25.4f/56};
const int NUM_ROS_IMP = 21;
constexpr int ROSCA_PAGE_SIZE = 6;  // items per page (2×3 grid)

// UI color palette (Figma Dark Mode)
constexpr uint16_t C_BG     = 0x18E4;  // #1F1F21 background (canvas)
constexpr uint16_t C_PANEL  = 0x18E4;  // #1F1F21 panel/card
constexpr uint16_t C_BTN    = 0x2966;  // #2A2D34 button OFF background
constexpr uint16_t C_MUTED  = 0x52AB;  // #55575D text OFF / inactive
constexpr uint16_t C_ACCENT = 0x5330;  // #556483 button ON / accent / border
constexpr uint16_t C_WHITE  = 0xFFFF;  // #FFFFFF text ON / borders

constexpr uint32_t SPLASH_DURATION = 5000;  // ms


enum class AppState : uint8_t {
  SPLASH,
  MAIN_MENU, AVANCE_MODE, AVANCE_SELECT, ROSCADO_MODE,
  ROSCA_SELECT, THREAD_UNIT_SWITCH, DIVISOR_MODE,
  CONFIG_MENU, CONFIG_DIRECTION, CONFIG_RATIO,
  MENU_OVERLAY, INFO_SCREEN, INTERFACE_SCREEN
};

enum class ButtonEvent : uint8_t { NONE, PRESSED };

struct Config {
  bool cwDirection;
  uint8_t leadScrewPitch;
  bool darkMode;
  Config() : cwDirection(true), leadScrewPitch(3), darkMode(true) {}
  void load() {
    if(EEPROM.read(EEPROM_MAGIC_ADDR) == EEPROM_MAGIC_VALUE) {
      cwDirection = EEPROM.read(EEPROM_DIRECTION_ADDR);
      leadScrewPitch = EEPROM.read(EEPROM_LEADSCREW_ADDR);
      if(leadScrewPitch < 1 || leadScrewPitch > 10) leadScrewPitch = 3;
      darkMode = EEPROM.read(EEPROM_DARKMODE_ADDR);
    } else {
      save();
    }
  }
  void save() {
    EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
    EEPROM.write(EEPROM_DIRECTION_ADDR, cwDirection);
    EEPROM.write(EEPROM_LEADSCREW_ADDR, leadScrewPitch);
    EEPROM.write(EEPROM_DARKMODE_ADDR, darkMode);
    EEPROM.commit();
  }
};

class Button {
private:
  uint8_t pin;
  uint32_t lastPress;
  bool lastState;
public:
  Button(uint8_t p): pin(p), lastPress(0), lastState(true) {
    pinMode(pin, INPUT_PULLUP);
  }
  ButtonEvent read() {
    bool cur = digitalRead(pin);
    uint32_t now = millis();
    ButtonEvent e = ButtonEvent::NONE;
    if(lastState && !cur){
      if(now - lastPress > DEBOUNCE_DELAY){
        e = ButtonEvent::PRESSED;
        lastPress = now;
      }
    }
    lastState = cur;
    return e;
  }
};

volatile int32_t encCount = 0;

void IRAM_ATTR isrA(){
  bool a = digitalRead(ENC_A);
  bool b = digitalRead(ENC_B);
  encCount += (a==b) ? 1 : -1;
}
void IRAM_ATTR isrB(){
  bool a = digitalRead(ENC_A);
  bool b = digitalRead(ENC_B);
  encCount += (a!=b) ? 1 : -1;
}

class DisplayManager {
private:
  Adafruit_ILI9341& tft;
public:
  DisplayManager(Adafruit_ILI9341& d): tft(d) {}
  
  void begin(){
    SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI, TFT_CS);
    tft.begin();
    tft.setRotation(1);
    tft.fillScreen(C_BG);
  }
  
  void mainMenu(int sel){
    tft.fillScreen(C_BG);

    // Background illustration — left side (193x240, x=0, y=0)
    tft.drawRGBBitmap(BG_MAIN_X, BG_MAIN_Y, bg_main, BG_MAIN_W, BG_MAIN_H);

    // Header: "| MAIN PAGE" — Orbitron 8pt, top-left
    tft.setFont(&Orbitron_Medium8pt7b);
    tft.setTextColor(C_WHITE);
    tft.setCursor(28, 33);
    tft.print("| MAIN PAGE");
    tft.setFont(nullptr);

    // Button-Menu icon — bitmap Off state, top-right (279,16) 24x24
    tft.drawRGBBitmap(BTN_MENU_X, BTN_MENU_Y, btn_menu_off, BTN_MENU_W, BTN_MENU_H);

    // Menu option buttons — right column, Orbitron 8pt
    const char* labels[3] = {"FEED", "THREAD", "DIVIDER"};
    const int btnY[3] = {65, 100, 135};
    const int bx = 210, bw = 110, bh = 25;

    tft.setFont(&Orbitron_Medium8pt7b);
    for(int i = 0; i < 3; i++){
      uint16_t bgColor   = (i == sel) ? C_ACCENT : C_BTN;
      uint16_t textColor = (i == sel) ? C_WHITE  : C_MUTED;
      tft.fillRect(bx, btnY[i], bw, bh, bgColor);
      tft.setTextColor(textColor);

      // Right-align label inside button (Figma: text align = right)
      int16_t x1, y1;
      uint16_t tw, th;
      tft.getTextBounds(labels[i], 0, 0, &x1, &y1, &tw, &th);
      int cx = bx + bw - (int)tw - x1 - 28;  // 28px padding from right edge
      int cy = btnY[i] + (bh - (int)th) / 2 - y1;
      tft.setCursor(cx, cy);
      tft.print(labels[i]);
    }
    tft.setFont(nullptr);
  }

  // Restore a screen rectangle from the bg_main PROGMEM bitmap.
  // Pixels outside the bitmap bounds are filled with C_BG.
  void refreshBgRegion(int x, int y, int w, int h){
    tft.startWrite();
    for(int row = 0; row < h; row++){
      for(int col = 0; col < w; col++){
        int sx = x + col, sy = y + row;
        uint16_t c = C_BG;
        if(sx < BG_MAIN_W && sy < BG_MAIN_H)
          c = pgm_read_word(&bg_main[sy * BG_MAIN_W + sx]);
        tft.writePixel(sx, sy, c);
      }
    }
    tft.endWrite();
  }

  void drawMainRPM(int rpm){
    tft.fillRect(0, 165, 300, 75, C_BG);

    char numBuf[8];
    snprintf(numBuf, sizeof(numBuf), "%d", rpm);
    tft.setFont(&Orbitron_Medium18pt7b);
    tft.setTextColor(C_WHITE);
    tft.setCursor(28, 215);
    tft.print(numBuf);
    tft.setFont(&Orbitron_Medium8pt7b);
    tft.setCursor(tft.getCursorX() + 4, 215);
    tft.print("/ rpm");
    tft.setFont(nullptr);
  }
  
  void configMenu(int sel){
    tft.fillScreen(C_BG);

    // Title — Orbitron 8pt, same position as all screens
    tft.setFont(&Orbitron_Medium8pt7b);
    tft.setTextColor(C_WHITE);
    tft.setCursor(28, 33);
    tft.print("< CONFIGURATION");
    tft.setFont(nullptr);

    // Button-Menu — ON (still inside menu)
    tft.drawRGBBitmap(BTN_MENU_X, BTN_MENU_Y, btn_menu_on, BTN_MENU_W, BTN_MENU_H);

    // Config options — Figma: x=80, w=240, h=25, y=65/100, Orbitron 8pt right-aligned
    const char* items[2] = {"DIRECTION", "LEAD SCREW PITCH"};
    const int OPT_X = 80, OPT_W = 240, OPT_H = 25;
    const int OPT_Y[2] = {65, 100};

    tft.setFont(&Orbitron_Medium8pt7b);
    for(int i = 0; i < 2; i++){
      bool isSel = (i == sel);
      tft.fillRect(OPT_X, OPT_Y[i], OPT_W, OPT_H, isSel ? C_ACCENT : C_BTN);
      tft.setTextColor(isSel ? C_WHITE : C_MUTED);
      int16_t x1, y1; uint16_t tw, th;
      tft.getTextBounds(items[i], 0, 0, &x1, &y1, &tw, &th);
      int textX = OPT_X + OPT_W - 28 - (int)tw - x1;
      int textY = OPT_Y[i] + (OPT_H - (int)th) / 2 - y1;
      tft.setCursor(textX, textY);
      tft.print(items[i]);
    }
    tft.setFont(nullptr);

    // Button-Back — bottom-right
    tft.drawRGBBitmap(262, 182, btn_back_off, BTN_BACK_W, BTN_BACK_H);
  }
  
  void configDirection(bool cwDir){
    tft.fillScreen(C_BG);

    // Title — Orbitron 8pt
    tft.setFont(&Orbitron_Medium8pt7b);
    tft.setTextColor(C_WHITE);
    tft.setCursor(28, 33);
    tft.print("< CONFIG / DIRECTION");
    tft.setFont(nullptr);

    // Button-Menu — ON (inside config hierarchy)
    tft.drawRGBBitmap(BTN_MENU_X, BTN_MENU_Y, btn_menu_on, BTN_MENU_W, BTN_MENU_H);

    // CW / CCW option buttons — Figma: x=68/168, y=85, w=83, h=60
    const char* opts[2] = {"CW", "CCW"};
    const int OPT_X[2] = {68, 168};
    const int OPT_Y = 85, OPT_W = 83, OPT_H = 60;

    tft.setFont(&Orbitron_Medium12pt7b);
    for(int i = 0; i < 2; i++){
      bool isSel = (i == 0) ? cwDir : !cwDir;
      tft.fillRoundRect(OPT_X[i], OPT_Y, OPT_W, OPT_H, 4, isSel ? C_ACCENT : C_BTN);
      tft.setTextColor(isSel ? C_WHITE : C_MUTED);
      int16_t x1, y1; uint16_t tw, th;
      tft.getTextBounds(opts[i], 0, 0, &x1, &y1, &tw, &th);
      int textX = OPT_X[i] + (OPT_W - (int)tw) / 2 - x1;
      int textY = OPT_Y + (OPT_H - (int)th) / 2 - y1;
      tft.setCursor(textX, textY);
      tft.print(opts[i]);
    }
    tft.setFont(nullptr);

    // Buttons — bottom-right (Back=cancel, Ok=save)
    tft.drawRGBBitmap(222, 182, btn_back_off, BTN_BACK_W, BTN_BACK_H);
    tft.drawRGBBitmap(262, 182, btn_ok_off, BTN_OK_W, BTN_OK_H);
  }
  
  void configRatio(uint8_t pitch, int digitSel){
    tft.fillScreen(C_BG);

    // Title — Orbitron 8pt
    tft.setFont(&Orbitron_Medium8pt7b);
    tft.setTextColor(C_WHITE);
    tft.setCursor(28, 33);
    tft.print("< CONF/LEAD SCREW PITCH");
    tft.setFont(nullptr);

    // Button-Menu — ON (inside config hierarchy)
    tft.drawRGBBitmap(BTN_MENU_X, BTN_MENU_Y, btn_menu_on, BTN_MENU_W, BTN_MENU_H);

    // Digit boxes — centered: x=75 (tens) x=165 (units), y=88, w=80, h=65
    // Gap between boxes: 10px — total box area: 170px, startX=(320-170)/2=75
    uint8_t d0 = (pitch / 10) % 10;
    uint8_t d1 = pitch % 10;
    const uint8_t digits[2] = {d0, d1};
    const int OPT_X[2] = {75, 165};
    const int OPT_Y = 88, OPT_W = 80, OPT_H = 65;

    tft.setFont(&Orbitron_Medium24pt7b);
    for(int i = 0; i < 2; i++){
      bool isSel = (i == digitSel);
      tft.fillRoundRect(OPT_X[i], OPT_Y, OPT_W, OPT_H, 4, isSel ? C_ACCENT : C_BTN);
      tft.setTextColor(isSel ? C_WHITE : C_MUTED);
      char buf[2]; snprintf(buf, sizeof(buf), "%d", digits[i]);
      int16_t x1, y1; uint16_t tw, th;
      tft.getTextBounds(buf, 0, 0, &x1, &y1, &tw, &th);
      int textX = OPT_X[i] + (OPT_W - (int)tw) / 2 - x1;
      int textY = OPT_Y + (OPT_H - (int)th) / 2 - y1;
      tft.setCursor(textX, textY);
      tft.print(buf);
    }
    tft.setFont(nullptr);

    // Stepper buttons — centered above/below the active digit box only
    // Up: 8px above box top (y=88-8-30=50), Down: 8px below box bottom (y=88+65+8=161)
    int stepperX = OPT_X[digitSel] + (OPT_W - BTN_STEPPER_W) / 2;
    tft.drawRGBBitmap(stepperX, 50, btn_stepper_up_off, BTN_STEPPER_W, BTN_STEPPER_H);
    tft.drawRGBBitmap(stepperX, 161, btn_stepper_down_off, BTN_STEPPER_W, BTN_STEPPER_H);

    // Buttons — bottom-right (Back=cancel, Ok=confirm digit / save)
    tft.drawRGBBitmap(222, 182, btn_back_off, BTN_BACK_W, BTN_BACK_H);
    tft.drawRGBBitmap(262, 182, btn_ok_off, BTN_OK_W, BTN_OK_H);
  }
  
  void drawAvanceSelect(int sel){
    tft.fillScreen(C_BG);

    // Header: "< FEED SELECTION" — Orbitron 8pt, top-left
    tft.setFont(&Orbitron_Medium8pt7b);
    tft.setTextColor(C_WHITE);
    tft.setCursor(28, 33);
    tft.print("< FEED SELECTION");
    tft.setFont(nullptr);

    // Button-Menu icon — top-right (same position as main screen)
    tft.drawRGBBitmap(BTN_MENU_X, BTN_MENU_Y, btn_menu_off, BTN_MENU_W, BTN_MENU_H);

    // Feed option grid — 2 cols x 3 rows (6 slots, 5 with values)
    // Figma: tab w=110 h=20, col_left x=40, col_right x=170, rows y=65/100/135
    const int COLS = 2;
    const int ROWS = 3;
    const int TAB_W = 110, TAB_H = 20;
    const int TAB_GAP_X = 20, TAB_GAP_Y = 15;
    const int GRID_X = 40, GRID_Y = 65;

    // Tab text: Figma=10px → Orbitron 8pt (Adafruit fonts render ~30% taller than stated)
    tft.setFont(&Orbitron_Medium8pt7b);
    for(int i = 0; i < COLS * ROWS; i++){
      int col = i % COLS;
      int row = i / COLS;
      int x = GRID_X + col * (TAB_W + TAB_GAP_X);
      int y = GRID_Y + row * (TAB_H + TAB_GAP_Y);

      if(i < NUM_AV){
        bool isSel = (i == sel);
        uint16_t bgColor = isSel ? C_ACCENT : C_BTN;
        uint16_t txColor = isSel ? C_WHITE  : C_MUTED;
        tft.fillRect(x, y, TAB_W, TAB_H, bgColor);
        tft.setTextColor(txColor);
        char buf[8];
        snprintf(buf, sizeof(buf), "%.2f", avances[i]);
        int16_t x1, y1; uint16_t tw, th;
        tft.getTextBounds(buf, 0, 0, &x1, &y1, &tw, &th);
        tft.setCursor(x + (TAB_W - (int)tw) / 2 - x1,
                      y + (TAB_H - (int)th) / 2 - y1);
        tft.print(buf);
      }
      // 6th slot: empty (C_BG background)
    }
    tft.setFont(nullptr);

    // Pagination bar — bottom-left
    // Figma: caret_left x=29 y=192, text x=44, caret_right x=77
    const int PAGE_SLOTS = COLS * ROWS;
    const int totalPages = (NUM_AV + PAGE_SLOTS - 1) / PAGE_SLOTS;
    // Solid caret-left (mirrored) — off on page 1 (no previous page)
    tft.drawRGBBitmap(17, 182, solid_left_off, SOLID_W, SOLID_H);
    // "1 / N" label — Figma=10px → Orbitron 8pt
    tft.setFont(&Orbitron_Medium8pt7b);
    tft.setTextColor(C_WHITE);
    char pagBuf[8];
    snprintf(pagBuf, sizeof(pagBuf), "1 / %d", totalPages);
    tft.setCursor(44, 199);
    tft.print(pagBuf);
    int afterPag = tft.getCursorX();
    tft.setFont(nullptr);
    // Solid caret-right — off when only 1 page
    tft.drawRGBBitmap(afterPag + 4, 182, solid_off, SOLID_W, SOLID_H);

    // Button-Back and Button-Ok — bottom-right (Figma: x=222/262, y=182)
    tft.drawRGBBitmap(222, 182, btn_back_off, BTN_BACK_W, BTN_BACK_H);
    tft.drawRGBBitmap(262, 182, btn_ok_off, BTN_OK_W, BTN_OK_H);
  }
  
  void drawThreadUnitSwitch(int sel){
    // sel: 0=METRIC, 1=UTS
    tft.fillScreen(C_BG);
    tft.setFont(&Orbitron_Medium8pt7b);
    tft.setTextColor(C_WHITE);
    tft.setCursor(28, 33);
    tft.print("< THREADS UNIT SWITCH");
    tft.setFont(nullptr);
    tft.drawRGBBitmap(BTN_MENU_X, BTN_MENU_Y, btn_menu_off, BTN_MENU_W, BTN_MENU_H);

    const char* labels[] = {"METRIC", "UTS"};
    const int OPT_Y[]    = {72, 117};
    tft.setFont(&Orbitron_Medium8pt7b);
    for(int i = 0; i < 2; i++){
      uint16_t bg = (sel == i) ? C_ACCENT : C_BTN;
      uint16_t fg = (sel == i) ? C_WHITE  : C_MUTED;
      tft.fillRoundRect(28, OPT_Y[i], 264, 30, 4, bg);
      tft.setTextColor(fg);
      int16_t x1, y1; uint16_t tw, th;
      tft.getTextBounds(labels[i], 0, 0, &x1, &y1, &tw, &th);
      tft.setCursor(28 + (264 - (int)tw) / 2 - x1, OPT_Y[i] + (30 + (int)th) / 2);
      tft.print(labels[i]);
    }
    tft.setFont(nullptr);
    tft.drawRGBBitmap(222, 182, btn_back_off, BTN_BACK_W, BTN_BACK_H);
    tft.drawRGBBitmap(262, 182, btn_ok_off,   BTN_OK_W,   BTN_OK_H);
  }

  // Generic thread selection page — handles any number of items and pages
  // page: 0-indexed page number | sel: absolute index in array | imperial: unit mode
  void drawRoscaPage(int page, int sel, bool imperial){
    tft.fillScreen(C_BG);
    tft.setFont(&Orbitron_Medium8pt7b);
    tft.setTextColor(C_WHITE);
    tft.setCursor(28, 33);
    tft.print(imperial ? "< THREAD SELECTION (tpi)" : "< THREAD SELECTION (mm)");
    tft.setFont(nullptr);
    tft.drawRGBBitmap(BTN_MENU_X, BTN_MENU_Y, btn_menu_off, BTN_MENU_W, BTN_MENU_H);

    const int tot         = imperial ? NUM_ROS_IMP : NUM_ROS;
    const int total_pages = (tot + ROSCA_PAGE_SIZE - 1) / ROSCA_PAGE_SIZE;
    const int start       = page * ROSCA_PAGE_SIZE;
    const int count       = min(ROSCA_PAGE_SIZE, tot - start);

    // Grid: 2×4 layout (8 items per page)
    const int COLS = 2;
    const int TAB_W = 110, TAB_H = 20, TAB_GAP_X = 20, TAB_GAP_Y = 15;
    const int GRID_X = 40, GRID_Y = 65;

    tft.setFont(&Orbitron_Medium8pt7b);
    for(int i = 0; i < count; i++){
      int idx = start + i;
      int col = i % COLS, row = i / COLS;
      int x = GRID_X + col * (TAB_W + TAB_GAP_X);
      int y = GRID_Y + row * (TAB_H + TAB_GAP_Y);
      bool isSel = (idx == sel);
      tft.fillRect(x, y, TAB_W, TAB_H, isSel ? C_ACCENT : C_BTN);
      tft.setTextColor(isSel ? C_WHITE : C_MUTED);
      char buf[12];
      if(imperial){
        float t = tpi_vals[idx];
        if(t == (float)(int)t) snprintf(buf, sizeof(buf), "%d", (int)t);
        else                   snprintf(buf, sizeof(buf), "%.1f", t);
      } else {
        snprintf(buf, sizeof(buf), "%.2f", roscas[idx]);
      }
      int16_t x1, y1; uint16_t tw, th;
      tft.getTextBounds(buf, 0, 0, &x1, &y1, &tw, &th);
      tft.setCursor(x + (TAB_W - (int)tw) / 2 - x1, y + (TAB_H - (int)th) / 2 - y1);
      tft.print(buf);
    }
    tft.setFont(nullptr);

    // Pagination — carets reflect available prev/next pages
    bool hasLeft  = (page > 0);
    bool hasRight = (page < total_pages - 1);
    tft.drawRGBBitmap(17, 182, hasLeft ? solid_left_on : solid_left_off, SOLID_W, SOLID_H);
    tft.setFont(&Orbitron_Medium8pt7b);
    tft.setTextColor(C_WHITE);
    tft.setCursor(44, 199);
    char pag[8]; snprintf(pag, sizeof(pag), "%d / %d", page + 1, total_pages);
    tft.print(pag);
    int afterPag = tft.getCursorX();
    tft.setFont(nullptr);
    tft.drawRGBBitmap(afterPag + 4, 182, hasRight ? solid_on : solid_off, SOLID_W, SOLID_H);

    tft.drawRGBBitmap(222, 182, btn_back_off, BTN_BACK_W, BTN_BACK_H);
    tft.drawRGBBitmap(262, 182, btn_ok_off,   BTN_OK_W,   BTN_OK_H);
  }
  
  void modeScreen(const char* title, float value, const char* unit){
    tft.fillScreen(C_BG);

    // Title — Orbitron 8pt, white, top-left (same position as all screens)
    tft.setFont(&Orbitron_Medium8pt7b);
    tft.setTextColor(C_WHITE);
    tft.setCursor(28, 33);
    tft.print(title);
    tft.setFont(nullptr);

    // Button-Menu — top-right (same position as all screens)
    tft.drawRGBBitmap(BTN_MENU_X, BTN_MENU_Y, btn_menu_off, BTN_MENU_W, BTN_MENU_H);

    // Value panel — Figma: x=30, y=81, w=260, h=60, rounded 6px, C_ACCENT bg
    const int PANEL_X = 30, PANEL_Y = 81, PANEL_W = 260, PANEL_H = 60;
    tft.fillRoundRect(PANEL_X, PANEL_Y, PANEL_W, PANEL_H, 6, C_ACCENT);

    // Measure number string (25pt) and unit string (9pt) to center together
    char numBuf[12];
    snprintf(numBuf, sizeof(numBuf), "%.2f", value);
    char unitBuf[12];
    snprintf(unitBuf, sizeof(unitBuf), "/ %s", unit);

    // Figma: number 40px → 24pt; unit 12px → 8pt (Adafruit renders larger than stated for big pt)
    tft.setFont(&Orbitron_Medium24pt7b);
    int16_t nx, ny; uint16_t nw, nh;
    tft.getTextBounds(numBuf, 0, 0, &nx, &ny, &nw, &nh);

    tft.setFont(&Orbitron_Medium8pt7b);
    int16_t ux, uy; uint16_t uw, uh;
    tft.getTextBounds(unitBuf, 0, 0, &ux, &uy, &uw, &uh);

    // Horizontal: center combined text in panel (6px gap between number and unit)
    const int GAP = 6;
    int totalW = (int)(nw - nx) + GAP + (int)(uw - ux);
    int startX  = PANEL_X + (PANEL_W - totalW) / 2;

    // Vertical: center number glyph in panel
    int panelCenterY = PANEL_Y + PANEL_H / 2;
    int baselineY    = panelCenterY - ny - (int)nh / 2;

    // Draw number in 24pt
    tft.setFont(&Orbitron_Medium24pt7b);
    tft.setTextColor(C_WHITE);
    tft.setCursor(startX - nx, baselineY);
    tft.print(numBuf);

    // Draw unit in 8pt on same baseline
    tft.setFont(&Orbitron_Medium8pt7b);
    tft.setCursor(tft.getCursorX() + GAP, baselineY);
    tft.print(unitBuf);
    tft.setFont(nullptr);

    // Button-Back — bottom-right (no Button-Ok in mode screens)
    tft.drawRGBBitmap(262, 182, btn_back_off, BTN_BACK_W, BTN_BACK_H);
  }

  // RPM display for active mode screens (Feed/Thread) — Figma style, solid C_BG background
  // RPM number: confirmed 25pt × 0.7 (Adafruit over-render compensation) = 17.5pt → 16pt; unit 8pt
  void drawModeRPM(int rpm){
    tft.fillRect(0, 165, 255, 75, C_BG);
    char numBuf[8];
    snprintf(numBuf, sizeof(numBuf), "%d", rpm);
    tft.setFont(&Orbitron_Medium18pt7b);
    tft.setTextColor(C_WHITE);
    tft.setCursor(28, 215);
    tft.print(numBuf);
    tft.setFont(&Orbitron_Medium8pt7b);
    tft.setCursor(tft.getCursorX() + 4, 215);
    tft.print("/ rpm");
    tft.setFont(nullptr);
  }

  // Divider static frame — call once on screen entry
  void divisorBase(){
    tft.fillScreen(C_BG);

    // Header: "< DIVIDER" — Orbitron 8pt, white, top-left
    tft.setFont(&Orbitron_Medium8pt7b);
    tft.setTextColor(C_WHITE);
    tft.setCursor(28, 33);
    tft.print("< DIVIDER");
    tft.setFont(nullptr);

    // Button-Menu — top-right
    tft.drawRGBBitmap(BTN_MENU_X, BTN_MENU_Y, btn_menu_off, BTN_MENU_W, BTN_MENU_H);

    // Value panel — Figma: x=30, y=81, w=260, h=60, rounded 6px, C_ACCENT bg
    tft.fillRoundRect(30, 81, 260, 60, 6, C_ACCENT);

    // Bottom-right buttons — Reload at (222,182), Back at (262,182)
    tft.drawRGBBitmap(222, 182, btn_reload_off, 30, 30);
    tft.drawRGBBitmap(262, 182, btn_back_off, BTN_BACK_W, BTN_BACK_H);
  }

  // Divider angle update — call every UPDATE_ANGLE ms
  // Repaints panel and redraws centered value (no static elements touched)
  void divisorAngle(float angle){
    tft.fillRoundRect(30, 81, 260, 60, 6, C_ACCENT);

    char numBuf[10];
    snprintf(numBuf, sizeof(numBuf), "%.2f", angle);
    const char* unitBuf = "/ deg";

    // Measure both strings — center combined text in full panel width (same as modeScreen)
    tft.setFont(&Orbitron_Medium24pt7b);
    int16_t nx, ny; uint16_t nw, nh;
    tft.getTextBounds(numBuf, 0, 0, &nx, &ny, &nw, &nh);

    tft.setFont(&Orbitron_Medium8pt7b);
    int16_t ux, uy; uint16_t uw, uh;
    tft.getTextBounds(unitBuf, 0, 0, &ux, &uy, &uw, &uh);

    const int PANEL_X = 30, PANEL_W = 260, GAP = 6;
    int totalW   = (int)(nw - nx) + GAP + (int)(uw - ux);
    int startX   = PANEL_X + (PANEL_W - totalW) / 2;
    int panelCenterY = 81 + 30;
    int baselineY    = panelCenterY - ny - (int)nh / 2;

    tft.setFont(&Orbitron_Medium24pt7b);
    tft.setTextColor(C_WHITE);
    tft.setCursor(startX - nx, baselineY);
    tft.print(numBuf);

    tft.setFont(&Orbitron_Medium8pt7b);
    tft.setCursor(tft.getCursorX() + GAP, baselineY);
    tft.print(unitBuf);
    tft.setFont(nullptr);
  }

  // ===== Splash screen =====
  void drawSplash() {
    tft.fillScreen(C_BG);
    tft.drawRGBBitmap(LOGO_X, LOGO_Y, logo_splash, LOGO_W, LOGO_H);
  }

  // ===== Menu overlay (Button-Menu) =====
  void drawMenuOverlay(int sel) {
    tft.fillScreen(C_BG);

    // Button-Menu — top-right (ON state: menu is open)
    tft.drawRGBBitmap(BTN_MENU_X, BTN_MENU_Y, btn_menu_on, BTN_MENU_W, BTN_MENU_H);

    // Menu options — Figma: x=140, w=180, h=25, y=65/100/135, Orbitron 8pt right-aligned
    const char* items[3] = {"CONFIGURATION", "INTERFACE", "INFO"};
    const int OPT_X = 140, OPT_W = 180, OPT_H = 25;
    const int OPT_Y[3] = {65, 100, 135};

    tft.setFont(&Orbitron_Medium8pt7b);
    for(int i = 0; i < 3; i++){
      bool isSel = (i == sel);
      tft.fillRect(OPT_X, OPT_Y[i], OPT_W, OPT_H, isSel ? C_ACCENT : C_BTN);
      tft.setTextColor(isSel ? C_WHITE : C_MUTED);
      int16_t x1, y1; uint16_t tw, th;
      tft.getTextBounds(items[i], 0, 0, &x1, &y1, &tw, &th);
      // Right-align: 28px padding from right edge of option
      int textX = OPT_X + OPT_W - 28 - (int)tw - x1;
      int textY = OPT_Y[i] + (OPT_H - (int)th) / 2 - y1;
      tft.setCursor(textX, textY);
      tft.print(items[i]);
    }
    tft.setFont(nullptr);

    // Button-Back — bottom-right
    tft.drawRGBBitmap(262, 182, btn_back_off, BTN_BACK_W, BTN_BACK_H);
  }

  // ===== Info screen =====
  void drawInfo(){
    tft.fillScreen(C_BG);

    tft.setFont(&Orbitron_Medium8pt7b);
    tft.setTextColor(C_WHITE);
    tft.setCursor(28, 33);
    tft.print("< INFO");

    tft.drawRGBBitmap(BTN_MENU_X, BTN_MENU_Y, btn_menu_on, BTN_MENU_W, BTN_MENU_H);

    tft.setFont(&Montserrat_Light6pt7b);
    tft.setTextColor(C_WHITE);
    const int TEXT_X = 28;
    int y = 65;

    tft.setCursor(TEXT_X, y); tft.print("TheMakerVoid ELS v1.0");
    y += 14;
    tft.setCursor(TEXT_X, y); tft.print("ESP32-S3 DevKit C-1");
    y += 12;
    tft.setCursor(TEXT_X, y); tft.print("ILI9341 TFT 320x240");
    y += 12;
    tft.setCursor(TEXT_X, y); tft.print("Lichuan A6 Servo");
    y += 12;
    tft.setCursor(TEXT_X, y); tft.print("Encoder 1440 p/rev");
    y += 14;
    tft.setCursor(TEXT_X, y); tft.print("Wokwi #452163489175406593");
    y += 14;
    tft.setCursor(TEXT_X, y); tft.print("This is an open-source, public domain");
    y += 11;
    tft.setCursor(TEXT_X, y); tft.print("project. Feel free to use it however");
    y += 11;
    tft.setCursor(TEXT_X, y); tft.print("you want. To buy the creator a beer:");
    y += 11;
    tft.setCursor(TEXT_X, y); tft.print("www.themakervoid/donation");

    tft.setFont(nullptr);

    tft.drawRGBBitmap(262, 182, btn_back_off, BTN_BACK_W, BTN_BACK_H);
  }

  // ===== Interface screen (dark mode toggle) =====
  void drawInterface(bool darkMode){
    tft.fillScreen(C_BG);

    tft.setFont(&Orbitron_Medium8pt7b);
    tft.setTextColor(C_WHITE);
    tft.setCursor(28, 33);
    tft.print("< INTERFACE");
    tft.setFont(nullptr);

    tft.drawRGBBitmap(BTN_MENU_X, BTN_MENU_Y, btn_menu_on, BTN_MENU_W, BTN_MENU_H);

    const int ROW_Y = 111;
    const int LABEL_X = 28;
    const int SLIDER_X = 262;
    const int SLIDER_Y = ROW_Y - BTN_SLIDER_H + 2;

    tft.setFont(&Orbitron_Medium8pt7b);
    tft.setTextColor(C_WHITE);
    tft.setCursor(LABEL_X, ROW_Y);
    tft.print("DARK MODE");
    tft.setFont(nullptr);

    tft.drawRGBBitmap(SLIDER_X, SLIDER_Y,
      darkMode ? btn_slider_on : btn_slider_off,
      BTN_SLIDER_W, BTN_SLIDER_H);

    tft.drawRGBBitmap(262, 182, btn_back_off, BTN_BACK_W, BTN_BACK_H);
  }
};

// ===== Shared variables Core0 ↔ Core1 =====
volatile AppState g_servoState    = AppState::SPLASH;
volatile int      g_avanceSel     = 0;
volatile int      g_roscaSel      = 0;
volatile uint8_t  g_leadPitch     = 3;
volatile bool     g_cwDirection   = true;
volatile bool     g_servoActive   = false;
volatile bool     g_imperialThread = false;

// ===== Servo control Core1 — private variables =====
static int32_t servoLastE_c1       = 0;
static int32_t servoAccumulator_c1 = 0;

void IRAM_ATTR updateServoCore1() {
  // Direct read of volatile variables — safe for types <= 32 bits on ESP32
  // portENTER_CRITICAL is not used in the hot loop to avoid contention with Core0
  bool     active = g_servoActive;
  if (!active) {
    servoLastE_c1       = encCount;
    servoAccumulator_c1 = 0;
    return;
  }

  AppState st    = g_servoState;
  int      avSel = g_avanceSel;
  int      roSel = g_roscaSel;
  uint8_t  pitch = g_leadPitch;
  bool     cw    = g_cwDirection;

  int32_t cur  = encCount;
  int32_t diff = cur - servoLastE_c1;
  if (diff == 0) return;
  servoLastE_c1 = cur;

  float F = (st == AppState::AVANCE_MODE) ? avances[avSel]
            : (g_imperialThread ? roscas_imperial[roSel] : roscas[roSel]);
  int32_t num = (int32_t)(F * 100.0f + 0.5f);
  int32_t den = (int32_t)pitch * 100;
  if (den == 0) return;

  bool    encForward = (diff > 0);
  int32_t absDiff    = abs(diff);

  servoAccumulator_c1 += absDiff * num;
  int32_t pulsesToSend = servoAccumulator_c1 / den;
  servoAccumulator_c1 -= pulsesToSend * den;

  if (pulsesToSend <= 0) return;

  bool forward = cw ? encForward : !encForward;

  digitalWrite(DIR_PIN, forward ? HIGH : LOW);
  ets_delay_us(2);  // Setup time DIR→PUL (Lichuan A6) — does not disable interrupts
  for (int32_t i = 0; i < pulsesToSend; i++) {
    digitalWrite(PUL_PIN, HIGH);
    ets_delay_us(3);  // Minimum pulse width Lichuan A6
    digitalWrite(PUL_PIN, LOW);
    ets_delay_us(3);  // Gap between pulses
  }
}

class Application {
private:
  Adafruit_ILI9341 tft;
  DisplayManager disp;
  Button bUp, bDown, bSel, bBack;
  AppState state;
  AppState prevState;
  int menuSel, configMenuSel, ratioDigitSel, avanceSel, roscaSel, roscaPage, threadUnitSel;
  int menuOverlaySel;
  float ang, lastAng;
  uint32_t tRPM, tANG, splashStart, lastTouchTime;
  int lastRPM;
  Config config;

public:
  Application()
  : tft(TFT_CS, TFT_DC, TFT_RST), disp(tft),
    bUp(BTN_UP), bDown(BTN_DOWN), bSel(BTN_SELECT), bBack(BTN_BACK),
    state(AppState::SPLASH), prevState(AppState::MAIN_MENU),
    menuSel(0), configMenuSel(0), ratioDigitSel(0),
    avanceSel(0), roscaSel(0), roscaPage(0), threadUnitSel(0), menuOverlaySel(0),
    ang(0.0f), lastAng(-1000.0f),
    tRPM(0), tANG(0), splashStart(0), lastTouchTime(0), lastRPM(0) {}

  void begin(){
    EEPROM.begin(512);
    config.load();
    disp.begin();
    tft.invertDisplay(!config.darkMode);
    disp.drawSplash();
    splashStart = millis();
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    attachInterrupt(ENC_A, isrA, CHANGE);
    attachInterrupt(ENC_B, isrB, CHANGE);
    pinMode(PUL_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(BRAKE_PIN, OUTPUT);  // GPIO reserved for brake (unused for now)
    pinMode(ENC_Z, INPUT_PULLUP);  // GPIO reserved for encoder Z signal (unused for now)
    digitalWrite(PUL_PIN, LOW);
    digitalWrite(BRAKE_PIN, LOW);  // Default state
    touch.begin();
    touch.setRotation(1);
  }

  void run(){
    uint32_t now = millis();

    // Splash auto-transition after SPLASH_DURATION
    if(state == AppState::SPLASH) {
      if(now - splashStart >= SPLASH_DURATION) {
        state = AppState::MAIN_MENU;
        disp.mainMenu(menuSel);
      }
      handleButtons();
      handleTouch();
      return;
    }

    if(state == AppState::MAIN_MENU || state == AppState::AVANCE_MODE || state == AppState::ROSCADO_MODE){
      if(now - tRPM >= UPDATE_RPM){
        updateRPM();
        if(state == AppState::MAIN_MENU)
          disp.drawMainRPM(lastRPM);
        else
          disp.drawModeRPM(lastRPM);
        tRPM = now;
      }
    }
    if(state==AppState::DIVISOR_MODE){
      if(now - tANG >= UPDATE_ANGLE){
        updateAngle();
        tANG = now;
      }
    }
    handleButtons();
    handleTouch();
    updateScreen();
  }

private:
  void updateRPM(){
    static int32_t last = 0;
    int32_t cur = encCount, diff = cur - last;
    last = cur;
    float rev = float(diff) / float(QUAD);
    float rpm = rev * (60000.0f / float(UPDATE_RPM));
    lastRPM = int(rpm);
  }

  void updateAngle(){
    int32_t cnt = encCount % QUAD;
    float rel = cnt * (360.0f / float(QUAD));
    if(rel < 0) rel += 360.0f;
    ang = rel;
  }

  uint8_t getDigit(uint8_t num, int pos){
    if(pos == 0) return (num / 10) % 10;
    return num % 10;
  }

  void setDigit(uint8_t& num, int pos, uint8_t val){
    if(pos == 0) num = (num % 10) + val * 10;
    else num = (num / 10) * 10 + val;
    if(num < 1) num = 1;
    if(num > 10) num = 10;
  }

  void handleButtons(){
    // Splash: any button skips to main
    if(state == AppState::SPLASH) {
      if(bUp.read()==ButtonEvent::PRESSED || bDown.read()==ButtonEvent::PRESSED ||
         bSel.read()==ButtonEvent::PRESSED || bBack.read()==ButtonEvent::PRESSED) {
        state = AppState::MAIN_MENU;
        disp.mainMenu(menuSel);
      }
      return;
    }

    if(state==AppState::MAIN_MENU){
      if(bUp.read()==ButtonEvent::PRESSED){
        menuSel = (menuSel + 2) % 3;
        disp.mainMenu(menuSel);
      }
      if(bDown.read()==ButtonEvent::PRESSED){
        menuSel = (menuSel + 1) % 3;
        disp.mainMenu(menuSel);
      }
      if(bSel.read()==ButtonEvent::PRESSED){
        if(menuSel == 0){
          state = AppState::AVANCE_SELECT;
          disp.drawAvanceSelect(avanceSel);
        }
        else if(menuSel == 1){
          roscaSel = 0;
          threadUnitSel = 0;
          state = AppState::THREAD_UNIT_SWITCH;
          disp.drawThreadUnitSwitch(threadUnitSel);
        }
        else if(menuSel == 2){
          state = AppState::DIVISOR_MODE;
          disp.divisorBase();
        }
      }
      if(bBack.read()==ButtonEvent::PRESSED){
        prevState = AppState::MAIN_MENU;
        menuOverlaySel = 0;
        state = AppState::MENU_OVERLAY;
        disp.drawMenuOverlay(menuOverlaySel);
      }
    }
    else if(state==AppState::THREAD_UNIT_SWITCH){
      if(bUp.read()==ButtonEvent::PRESSED || bDown.read()==ButtonEvent::PRESSED){
        threadUnitSel = 1 - threadUnitSel;
        disp.drawThreadUnitSwitch(threadUnitSel);
      }
      if(bSel.read()==ButtonEvent::PRESSED){
        roscaSel = 0;
        roscaPage = 0;
        state = AppState::ROSCA_SELECT;
        disp.drawRoscaPage(roscaPage, roscaSel, threadUnitSel == 1);
      }
    }
    else if(state==AppState::MENU_OVERLAY){
      if(bUp.read()==ButtonEvent::PRESSED){
        menuOverlaySel = (menuOverlaySel + 2) % 3;
        disp.drawMenuOverlay(menuOverlaySel);
      }
      if(bDown.read()==ButtonEvent::PRESSED){
        menuOverlaySel = (menuOverlaySel + 1) % 3;
        disp.drawMenuOverlay(menuOverlaySel);
      }
      if(bSel.read()==ButtonEvent::PRESSED){
        if(menuOverlaySel == 0){
          state = AppState::CONFIG_MENU;
          disp.configMenu(configMenuSel);
        }
        else if(menuOverlaySel == 1){
          state = AppState::INTERFACE_SCREEN;
          disp.drawInterface(config.darkMode);
        }
        else if(menuOverlaySel == 2){
          state = AppState::INFO_SCREEN;
          disp.drawInfo();
        }
      }
      if(bBack.read()==ButtonEvent::PRESSED){
        state = prevState;
        disp.mainMenu(menuSel);
      }
    }
    else if(state==AppState::CONFIG_MENU){
      if(bUp.read()==ButtonEvent::PRESSED){
        configMenuSel--;
        if(configMenuSel < 0) configMenuSel = 1;
        disp.configMenu(configMenuSel);
      }
      if(bDown.read()==ButtonEvent::PRESSED){
        configMenuSel++;
        if(configMenuSel > 1) configMenuSel = 0;
        disp.configMenu(configMenuSel);
      }
      if(bSel.read()==ButtonEvent::PRESSED){
        if(configMenuSel == 0){
          state = AppState::CONFIG_DIRECTION;
          disp.configDirection(config.cwDirection);
        } else {
          state = AppState::CONFIG_RATIO;
          ratioDigitSel = 0;
          disp.configRatio(config.leadScrewPitch, ratioDigitSel);
        }
      }
    }
    else if(state==AppState::CONFIG_DIRECTION){
      if(bUp.read()==ButtonEvent::PRESSED || bDown.read()==ButtonEvent::PRESSED){
        config.cwDirection = !config.cwDirection;
        disp.configDirection(config.cwDirection);
      }
      if(bSel.read()==ButtonEvent::PRESSED){
        config.save();
        g_cwDirection = config.cwDirection;
        state = AppState::CONFIG_MENU;
        disp.configMenu(configMenuSel);
      }
    }
    else if(state==AppState::CONFIG_RATIO){
      if(bSel.read()==ButtonEvent::PRESSED){
        ratioDigitSel++;
        if(ratioDigitSel > 1){
          config.save();
          g_leadPitch = config.leadScrewPitch;
          state = AppState::CONFIG_MENU;
          disp.configMenu(configMenuSel);
        } else {
          disp.configRatio(config.leadScrewPitch, ratioDigitSel);
        }
      }
      if(bUp.read()==ButtonEvent::PRESSED){
        uint8_t d = getDigit(config.leadScrewPitch, ratioDigitSel);
        d++;
        if(d > 9) d = 0;
        setDigit(config.leadScrewPitch, ratioDigitSel, d);
        disp.configRatio(config.leadScrewPitch, ratioDigitSel);
      }
      if(bDown.read()==ButtonEvent::PRESSED){
        uint8_t d = getDigit(config.leadScrewPitch, ratioDigitSel);
        if(d == 0) d = 9; else d--;
        setDigit(config.leadScrewPitch, ratioDigitSel, d);
        disp.configRatio(config.leadScrewPitch, ratioDigitSel);
      }
    }
    else if(state==AppState::AVANCE_SELECT){
      if(bUp.read()==ButtonEvent::PRESSED){
        avanceSel--;
        if(avanceSel < 0) avanceSel = NUM_AV - 1;
        disp.drawAvanceSelect(avanceSel);
      }
      if(bDown.read()==ButtonEvent::PRESSED){
        avanceSel++;
        if(avanceSel >= NUM_AV) avanceSel = 0;
        disp.drawAvanceSelect(avanceSel);
      }
      if(bSel.read()==ButtonEvent::PRESSED){
        state = AppState::AVANCE_MODE;
        g_servoState  = AppState::AVANCE_MODE;
        g_avanceSel   = avanceSel;
        g_roscaSel    = roscaSel;
        g_leadPitch   = config.leadScrewPitch;
        g_cwDirection = config.cwDirection;
        g_servoActive = true;
        disp.modeScreen("< FEED MODE", avances[avanceSel], "mm");
      }
    }
    else if(state==AppState::ROSCA_SELECT){
      bool imp     = (threadUnitSel == 1);
      int  tot     = imp ? NUM_ROS_IMP : NUM_ROS;
      int  pages   = (tot + ROSCA_PAGE_SIZE - 1) / ROSCA_PAGE_SIZE;
      int  pStart  = roscaPage * ROSCA_PAGE_SIZE;
      int  pEnd    = min(pStart + ROSCA_PAGE_SIZE, tot) - 1;
      if(bUp.read()==ButtonEvent::PRESSED){
        if(roscaSel > pStart){
          roscaSel--;
        } else if(roscaPage > 0){
          roscaPage--;
          roscaSel = min((roscaPage + 1) * ROSCA_PAGE_SIZE, tot) - 1;
        } else {
          roscaPage = pages - 1;
          roscaSel  = tot - 1;
        }
        disp.drawRoscaPage(roscaPage, roscaSel, imp);
      }
      if(bDown.read()==ButtonEvent::PRESSED){
        if(roscaSel < pEnd){
          roscaSel++;
        } else if(roscaPage < pages - 1){
          roscaPage++;
          roscaSel = roscaPage * ROSCA_PAGE_SIZE;
        } else {
          roscaPage = 0;
          roscaSel  = 0;
        }
        disp.drawRoscaPage(roscaPage, roscaSel, imp);
      }
      if(bSel.read()==ButtonEvent::PRESSED){
        float pitchVal = imp ? roscas_imperial[roscaSel] : roscas[roscaSel];
        float dispVal  = imp ? tpi_vals[roscaSel]        : roscas[roscaSel];
        state            = AppState::ROSCADO_MODE;
        g_servoState     = AppState::ROSCADO_MODE;
        g_avanceSel      = avanceSel;
        g_roscaSel       = roscaSel;
        g_leadPitch      = config.leadScrewPitch;
        g_cwDirection    = config.cwDirection;
        g_imperialThread = imp;
        g_servoActive    = true;
        disp.modeScreen("< THREAD MODE", dispVal, imp ? "TPI" : "Pitch");
      }
    }
    else if(state==AppState::DIVISOR_MODE){
      if(bSel.read()==ButtonEvent::PRESSED){
        encCount = 0;
        lastAng = -1000.0f;
      }
    }
    else if(state==AppState::INTERFACE_SCREEN){
      if(bSel.read()==ButtonEvent::PRESSED){
        config.darkMode = !config.darkMode;
        tft.invertDisplay(!config.darkMode);
        disp.drawInterface(config.darkMode);
      }
    }

    if(bBack.read()==ButtonEvent::PRESSED &&
       state!=AppState::SPLASH &&
       state!=AppState::MAIN_MENU &&
       state!=AppState::MENU_OVERLAY){
      if(state==AppState::ROSCA_SELECT){
        bool imp = (threadUnitSel == 1);
        if(roscaPage > 0){
          roscaPage--;
          roscaSel = roscaPage * ROSCA_PAGE_SIZE;
          disp.drawRoscaPage(roscaPage, roscaSel, imp);
        } else {
          state = AppState::THREAD_UNIT_SWITCH;
          disp.drawThreadUnitSwitch(threadUnitSel);
        }
      }
      else if(state==AppState::CONFIG_DIRECTION || state==AppState::CONFIG_RATIO){
        state = AppState::CONFIG_MENU;
        disp.configMenu(configMenuSel);
      }
      else if(state==AppState::INFO_SCREEN){
        state = AppState::MENU_OVERLAY;
        disp.drawMenuOverlay(menuOverlaySel);
      }
      else if(state==AppState::INTERFACE_SCREEN){
        config.save();
        state = AppState::MENU_OVERLAY;
        disp.drawMenuOverlay(menuOverlaySel);
        tft.invertDisplay(!config.darkMode);
      }
      else {
        state = AppState::MAIN_MENU;
        g_servoActive = false;
        disp.mainMenu(menuSel);
      }
    }
  }

  // ===== Touch hit-test helpers =====
  bool hitRect(int tx, int ty, int rx, int ry, int rw, int rh){
    return tx >= rx && tx <= rx + rw && ty >= ry && ty <= ry + rh;
  }

  // ===== handleTouch() — Redesigned UI touch zones =====
  // Bitmap button positions (from DisplayManager Figma layout):
  //   btn_menu:    (279, 16)  24×24   — all screens
  //   btn_back:    (262, 182) 30×30   — sub-screens (solo) or (222,182) when paired with btn_ok
  //   btn_ok:      (262, 182) 30×30   — select/config screens (always right of btn_back)
  //   btn_reload:  (222, 182) 30×30   — divider only
  //   btn_slider:  (262, 111) 21×13   — interface screen
  void handleTouch(){
    if(!touch.touched()) return;
    uint32_t now = millis();
    if(now - lastTouchTime < TOUCH_DEBOUNCE) return;

    TS_Point p = touch.getPoint();
    int x = map(p.x, TS_MAXX, TS_MINX, 0, 319);
    int y = map(p.y, TS_MAXY, TS_MINY, 0, 239);
    x = constrain(x, 0, 319);
    y = constrain(y, 0, 239);
    lastTouchTime = now;

    // --- SPLASH: any touch → main menu ---
    if(state == AppState::SPLASH){
      state = AppState::MAIN_MENU;
      disp.mainMenu(menuSel);
      return;
    }

    // --- MENU OVERLAY ---
    if(state == AppState::MENU_OVERLAY){
      // Options: CONFIGURATION / INTERFACE / INFO — x=140, y=65/100/135, w=180, h=25
      for(int i = 0; i < 3; i++){
        if(hitRect(x, y, 140, 65 + i * 35, 180, 25)){
          menuOverlaySel = i;
          if(i == 0){
            state = AppState::CONFIG_MENU;
            disp.configMenu(configMenuSel);
          } else if(i == 1){
            state = AppState::INTERFACE_SCREEN;
            disp.drawInterface(config.darkMode);
          } else {
            state = AppState::INFO_SCREEN;
            disp.drawInfo();
          }
          return;
        }
      }
      // btn_back (262, 182) 30×30 → return to previous state
      if(hitRect(x, y, 258, 178, 38, 38)){
        state = prevState;
        disp.mainMenu(menuSel);
        return;
      }
      return;
    }

    // --- btn_menu touch (top-right, most screens) → open menu overlay ---
    // Screens that should NOT open overlay: SPLASH, MENU_OVERLAY (handled above)
    if(state != AppState::SPLASH && hitRect(x, y, 275, 12, 38, 38)){
      prevState = state;
      menuOverlaySel = 0;
      state = AppState::MENU_OVERLAY;
      disp.drawMenuOverlay(menuOverlaySel);
      return;
    }

    // --- MAIN MENU ---
    if(state == AppState::MAIN_MENU){
      // 3 options right column: x=210, y=65/100/135, w=110, h=25
      const int optY[] = {65, 100, 135};
      for(int i = 0; i < 3; i++){
        if(hitRect(x, y, 210, optY[i], 110, 25)){
          menuSel = i;
          if(i == 0){
            state = AppState::AVANCE_SELECT;
            disp.drawAvanceSelect(avanceSel);
          } else if(i == 1){
            roscaSel = 0;
            threadUnitSel = 0;
            state = AppState::THREAD_UNIT_SWITCH;
            disp.drawThreadUnitSwitch(threadUnitSel);
          } else {
            state = AppState::DIVISOR_MODE;
            disp.divisorBase();
          }
          return;
        }
      }
      return;
    }

    // --- THREAD UNIT SWITCH ---
    if(state == AppState::THREAD_UNIT_SWITCH){
      // METRIC / UTS options: x=28, y=72/117, w=264, h=30
      if(hitRect(x, y, 28, 72, 264, 30)){
        threadUnitSel = 0;
        disp.drawThreadUnitSwitch(threadUnitSel);
        return;
      }
      if(hitRect(x, y, 28, 117, 264, 30)){
        threadUnitSel = 1;
        disp.drawThreadUnitSwitch(threadUnitSel);
        return;
      }
      // btn_ok (262, 182) → confirm and go to ROSCA_SELECT
      if(hitRect(x, y, 258, 178, 38, 38)){
        roscaSel = 0;
        roscaPage = 0;
        state = AppState::ROSCA_SELECT;
        disp.drawRoscaPage(roscaPage, roscaSel, threadUnitSel == 1);
        return;
      }
      // btn_back (222, 182) → return to MAIN_MENU
      if(hitRect(x, y, 218, 178, 38, 38)){
        state = AppState::MAIN_MENU;
        disp.mainMenu(menuSel);
        return;
      }
      return;
    }

    // --- FEED SELECT ---
    if(state == AppState::AVANCE_SELECT){
      // Tab grid 2×3: x=40/170, y=65/100/135, w=110, h=20
      const int tabX[] = {40, 170};
      const int tabY[] = {65, 100, 135};
      for(int i = 0; i < NUM_AV; i++){
        int col = i % 2, row = i / 2;
        if(hitRect(x, y, tabX[col], tabY[row], 110, 20)){
          avanceSel = i;
          disp.drawAvanceSelect(avanceSel);
          return;
        }
      }
      // btn_ok (262, 182) → confirm selection
      if(hitRect(x, y, 258, 178, 38, 38)){
        state = AppState::AVANCE_MODE;
        g_servoState  = AppState::AVANCE_MODE;
        g_avanceSel   = avanceSel;
        g_roscaSel    = roscaSel;
        g_leadPitch   = config.leadScrewPitch;
        g_cwDirection = config.cwDirection;
        g_servoActive = true;
        disp.modeScreen("< FEED MODE", avances[avanceSel], "mm");
        return;
      }
      // btn_back (222, 182) → return to MAIN_MENU
      if(hitRect(x, y, 218, 178, 38, 38)){
        state = AppState::MAIN_MENU;
        disp.mainMenu(menuSel);
        return;
      }
      return;
    }

    // --- ROSCA SELECT (paginated) ---
    if(state == AppState::ROSCA_SELECT){
      bool imp    = (threadUnitSel == 1);
      int  tot    = imp ? NUM_ROS_IMP : NUM_ROS;
      int  pages  = (tot + ROSCA_PAGE_SIZE - 1) / ROSCA_PAGE_SIZE;
      int  start  = roscaPage * ROSCA_PAGE_SIZE;
      int  count  = min(ROSCA_PAGE_SIZE, tot - start);

      // Tab grid 2×3: x=40/170, y=65/100/135, w=110, h=20
      const int tabX[] = {40, 170};
      const int tabY[] = {65, 100, 135};
      for(int i = 0; i < count; i++){
        int col = i % 2, row = i / 2;
        if(hitRect(x, y, tabX[col], tabY[row], 110, 20)){
          roscaSel = start + i;
          disp.drawRoscaPage(roscaPage, roscaSel, imp);
          return;
        }
      }
      // Left pagination caret (17, 182) 24×24 — previous page
      if(roscaPage > 0 && hitRect(x, y, 13, 178, 32, 32)){
        roscaPage--;
        roscaSel = roscaPage * ROSCA_PAGE_SIZE;
        disp.drawRoscaPage(roscaPage, roscaSel, imp);
        return;
      }
      // Right pagination caret — next page (approximate position after text)
      if(roscaPage < pages - 1 && hitRect(x, y, 90, 178, 32, 32)){
        roscaPage++;
        roscaSel = roscaPage * ROSCA_PAGE_SIZE;
        disp.drawRoscaPage(roscaPage, roscaSel, imp);
        return;
      }
      // btn_ok (262, 182) → confirm selection
      if(hitRect(x, y, 258, 178, 38, 38)){
        float dispVal = imp ? tpi_vals[roscaSel] : roscas[roscaSel];
        state            = AppState::ROSCADO_MODE;
        g_servoState     = AppState::ROSCADO_MODE;
        g_avanceSel      = avanceSel;
        g_roscaSel       = roscaSel;
        g_leadPitch      = config.leadScrewPitch;
        g_cwDirection    = config.cwDirection;
        g_imperialThread = imp;
        g_servoActive    = true;
        disp.modeScreen("< THREAD MODE", dispVal, imp ? "TPI" : "Pitch");
        return;
      }
      // btn_back (222, 182) → previous page or THREAD_UNIT_SWITCH
      if(hitRect(x, y, 218, 178, 38, 38)){
        if(roscaPage > 0){
          roscaPage--;
          roscaSel = roscaPage * ROSCA_PAGE_SIZE;
          disp.drawRoscaPage(roscaPage, roscaSel, imp);
        } else {
          state = AppState::THREAD_UNIT_SWITCH;
          disp.drawThreadUnitSwitch(threadUnitSel);
        }
        return;
      }
      return;
    }

    // --- AVANCE MODE / ROSCADO MODE ---
    if(state == AppState::AVANCE_MODE || state == AppState::ROSCADO_MODE){
      // btn_back (262, 182) → return to MAIN_MENU, stop servo
      if(hitRect(x, y, 258, 178, 38, 38)){
        state = AppState::MAIN_MENU;
        g_servoActive = false;
        disp.mainMenu(menuSel);
        return;
      }
      return;
    }

    // --- DIVISOR MODE ---
    if(state == AppState::DIVISOR_MODE){
      // btn_reload (222, 182) 30×30 → reset angle
      if(hitRect(x, y, 218, 178, 38, 38)){
        encCount = 0;
        lastAng = -1000.0f;
        return;
      }
      // btn_back (262, 182) → return to MAIN_MENU
      if(hitRect(x, y, 258, 178, 38, 38)){
        state = AppState::MAIN_MENU;
        disp.mainMenu(menuSel);
        return;
      }
      return;
    }

    // --- CONFIG MENU ---
    if(state == AppState::CONFIG_MENU){
      // Options: DIRECTION / LEAD SCREW PITCH — x=80, y=65/100, w=240, h=25
      if(hitRect(x, y, 80, 65, 240, 25)){
        configMenuSel = 0;
        state = AppState::CONFIG_DIRECTION;
        disp.configDirection(config.cwDirection);
        return;
      }
      if(hitRect(x, y, 80, 100, 240, 25)){
        configMenuSel = 1;
        state = AppState::CONFIG_RATIO;
        ratioDigitSel = 0;
        disp.configRatio(config.leadScrewPitch, ratioDigitSel);
        return;
      }
      // btn_back (262, 182) → return to MENU_OVERLAY
      if(hitRect(x, y, 258, 178, 38, 38)){
        state = AppState::MENU_OVERLAY;
        disp.drawMenuOverlay(menuOverlaySel);
        return;
      }
      return;
    }

    // --- CONFIG DIRECTION ---
    if(state == AppState::CONFIG_DIRECTION){
      // CW button: x=68, y=85, w=83, h=60
      if(hitRect(x, y, 68, 85, 83, 60)){
        config.cwDirection = true;
        disp.configDirection(config.cwDirection);
        return;
      }
      // CCW button: x=168, y=85, w=83, h=60
      if(hitRect(x, y, 168, 85, 83, 60)){
        config.cwDirection = false;
        disp.configDirection(config.cwDirection);
        return;
      }
      // btn_ok (262, 182) → save and return to CONFIG_MENU
      if(hitRect(x, y, 258, 178, 38, 38)){
        config.save();
        g_cwDirection = config.cwDirection;
        state = AppState::CONFIG_MENU;
        disp.configMenu(configMenuSel);
        return;
      }
      // btn_back (222, 182) → cancel and return to CONFIG_MENU
      if(hitRect(x, y, 218, 178, 38, 38)){
        state = AppState::CONFIG_MENU;
        disp.configMenu(configMenuSel);
        return;
      }
      return;
    }

    // --- CONFIG RATIO ---
    if(state == AppState::CONFIG_RATIO){
      // Digit box tens: x=75, y=88, w=80, h=65 → select digit 0
      if(hitRect(x, y, 75, 88, 80, 65)){
        ratioDigitSel = 0;
        disp.configRatio(config.leadScrewPitch, ratioDigitSel);
        return;
      }
      // Digit box units: x=165, y=88, w=80, h=65 → select digit 1
      if(hitRect(x, y, 165, 88, 80, 65)){
        ratioDigitSel = 1;
        disp.configRatio(config.leadScrewPitch, ratioDigitSel);
        return;
      }
      // Stepper up arrow (above active digit, y~50, 30×30)
      {
        int stepX = (ratioDigitSel == 0) ? 75 + (80 - BTN_STEPPER_W) / 2
                                         : 165 + (80 - BTN_STEPPER_W) / 2;
        if(hitRect(x, y, stepX - 4, 46, 38, 38)){
          uint8_t d = getDigit(config.leadScrewPitch, ratioDigitSel);
          d++; if(d > 9) d = 0;
          setDigit(config.leadScrewPitch, ratioDigitSel, d);
          disp.configRatio(config.leadScrewPitch, ratioDigitSel);
          return;
        }
        // Stepper down arrow (below active digit, y~161, 30×30)
        if(hitRect(x, y, stepX - 4, 157, 38, 38)){
          uint8_t d = getDigit(config.leadScrewPitch, ratioDigitSel);
          if(d == 0) d = 9; else d--;
          setDigit(config.leadScrewPitch, ratioDigitSel, d);
          disp.configRatio(config.leadScrewPitch, ratioDigitSel);
          return;
        }
      }
      // btn_ok (262, 182) → advance digit or save
      if(hitRect(x, y, 258, 178, 38, 38)){
        ratioDigitSel++;
        if(ratioDigitSel > 1){
          config.save();
          g_leadPitch = config.leadScrewPitch;
          state = AppState::CONFIG_MENU;
          disp.configMenu(configMenuSel);
        } else {
          disp.configRatio(config.leadScrewPitch, ratioDigitSel);
        }
        return;
      }
      // btn_back (222, 182) → cancel and return to CONFIG_MENU
      if(hitRect(x, y, 218, 178, 38, 38)){
        state = AppState::CONFIG_MENU;
        disp.configMenu(configMenuSel);
        return;
      }
      return;
    }

    // --- INTERFACE SCREEN ---
    if(state == AppState::INTERFACE_SCREEN){
      // btn_slider (262, 111) 21×13 or label area — toggle dark mode
      if(hitRect(x, y, 258, 105, 30, 24)){
        config.darkMode = !config.darkMode;
        tft.invertDisplay(!config.darkMode);
        disp.drawInterface(config.darkMode);
        return;
      }
      // btn_back (262, 182) → save and return to MENU_OVERLAY
      if(hitRect(x, y, 258, 178, 38, 38)){
        config.save();
        state = AppState::MENU_OVERLAY;
        disp.drawMenuOverlay(menuOverlaySel);
        tft.invertDisplay(!config.darkMode);
        return;
      }
      return;
    }

    // --- INFO SCREEN ---
    if(state == AppState::INFO_SCREEN){
      // btn_back (262, 182) → return to MENU_OVERLAY
      if(hitRect(x, y, 258, 178, 38, 38)){
        state = AppState::MENU_OVERLAY;
        disp.drawMenuOverlay(menuOverlaySel);
        return;
      }
      return;
    }
  }

  void updateScreen(){
    if(state==AppState::DIVISOR_MODE){
      if(fabs(ang - lastAng) >= 0.01f){
        disp.divisorAngle(ang);
        lastAng = ang;
      }
    }
  }
};

Application app;

void loop(){
  app.run();
}

// ===== Core1: dedicated FreeRTOS servo task (ESP32) =====
void servoTask(void* pvParameters) {
  servoLastE_c1 = encCount;  // Sync initial position
  for (;;) {
    updateServoCore1();
    vTaskDelay(1);  // Yield to the scheduler — avoids WDT and lets Core1 service other tasks
  }
}

void setup(){
  Serial.begin(115200);
  app.begin();
  // Launch the servo task on Core1 with high priority (24) and a 2KB stack
  xTaskCreatePinnedToCore(
    servoTask,    // task function
    "ServoTask",  // name (debug)
    4096,         // stack in bytes (float + PROGMEM arrays need enough stack)
    nullptr,      // parameter
    24,           // priority (0=min, 25=max on ESP32)
    nullptr,      // handle (not needed)
    1             // Core1
  );
}