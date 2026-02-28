#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <rom/ets_sys.h>   // ets_delay_us() — delay sin deshabilitar interrupciones (Core1)
// #include <XPT2046_Touchscreen.h>  // TODO: re-enable for real hardware (uncomment + uncomment handleTouch())
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
#define BRAKE_PIN 3  // GPIO para control del freno electromagnético
#define ENC_Z 2      // GPIO para señal Z (índice) del encoder - reservado

// XPT2046_Touchscreen touch(TOUCH_CS, TOUCH_IRQ);  // Disabled: Wokwi does not support XPT2046

#define EEPROM_MAGIC_ADDR 0
#define EEPROM_DIRECTION_ADDR 1
#define EEPROM_LEADSCREW_ADDR 2
#define EEPROM_DARKMODE_ADDR 3
#define EEPROM_MAGIC_VALUE 0xAC  // Incrementado para forzar re-init con darkMode

constexpr int QUAD = 1440;
constexpr float LEAD = 3.0f;
constexpr uint16_t UPDATE_RPM = 200;
constexpr uint16_t UPDATE_ANGLE = 50;
constexpr uint16_t DEBOUNCE_DELAY = 200;
// constexpr int TS_MINX = 395;   // Touch calibration — disabled for Wokwi
// constexpr int TS_MAXX = 3846;
// constexpr int TS_MINY = 412;
// constexpr int TS_MAXY = 3739;

const float avances[] = {0.07f, 0.10f, 0.13f, 0.18f, 0.25f};
const int NUM_AV = 5;
const float roscas[] = {0.40f, 0.50f, 0.70f, 0.80f, 1.00f, 1.25f, 1.50f, 1.75f, 2.00f, 2.50f, 3.00f, 3.50f, 4.00f};
const int NUM_ROS = 13;
constexpr int ROSCA_P1 = 6;

// UI color palette (Figma Dark Mode)
constexpr uint16_t C_BG     = 0x18E4;  // #1F1F21 background (canvas)
constexpr uint16_t C_PANEL  = 0x18E4;  // #1F1F21 panel/card
constexpr uint16_t C_BTN    = 0x2966;  // #2A2D34 button OFF background
constexpr uint16_t C_MUTED  = 0x52AB;  // #55575D text OFF / inactive
constexpr uint16_t C_ACCENT = 0x5330;  // #556483 button ON / accent / border
constexpr uint16_t C_WHITE  = 0xFFFF;  // #FFFFFF text ON / borders

constexpr uint32_t SPLASH_DURATION = 5000;  // ms

constexpr int GRID_START_Y = 60;
constexpr int BTN_H = 26;
constexpr int BTN_GAP_Y = 6;
constexpr int BTN_W = 130;
constexpr int BTN_LEFT_X = 15;
constexpr int BTN_RIGHT_X = 175;
constexpr int ARROW_Y = 190;
constexpr int ARROW_W = 50;
constexpr int ARROW_H = 24;
constexpr int CONF_W = 133;
constexpr int CONF_H = 24;
constexpr int CONF_X = (320 - CONF_W) / 2;
constexpr int CONF_Y = 190;

enum class AppState : uint8_t {
  SPLASH,
  MAIN_MENU, AVANCE_MODE, AVANCE_SELECT, ROSCADO_MODE,
  ROSCA_SELECT_P1, ROSCA_SELECT_P2, DIVISOR_MODE,
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
  
  void hud(){
    tft.fillRect(0,0,320,80, C_BG);
    tft.drawRect(0,0,320,80, ILI9341_DARKGREY);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_CYAN);
    tft.setCursor(10,10);
    tft.print("RPM");
  }
  
  void drawRPM(int rpm){
    tft.fillRect(118,16,84,34, C_BG);
    tft.setTextSize(3);
    tft.setTextColor(ILI9341_GREEN);
    char b[10];
    snprintf(b,sizeof(b),"%d",rpm);
    int off = 80 - (int)strlen(b)*18;
    tft.setCursor(120+off,18);
    tft.print(b);
  }
  
  void drawBack(){
    uint16_t c = ILI9341_RED;
    int cx = 300, cy = 20, hh = 10, head = 14, body = 12;
    for(int y=-hh; y<=hh; y++){
      int w = head - abs(y)*head/hh;
      int xs = cx - body - w;
      tft.drawFastHLine(xs, cy+y, w, c);
    }
    tft.fillRect(cx-body, cy-hh/2, body, hh, c);
  }
  
  void drawGearIcon(int cx, int cy, bool selected = false){
    uint16_t color = selected ? ILI9341_GREEN : ILI9341_YELLOW;
    int outerR = 18;
    int innerR = 10;
    int centerR = 5;
    int teethCount = 6;
    float rotationOffset = PI / 6.0;

    float tipWidth = 0.15 * 1.3 * 1.3;
    float baseWidth = 0.15 * 7.2;

    for(int i = 0; i < teethCount; i++){
      float angleCenter = i * 2.0 * PI / teethCount + rotationOffset;

      float angle1_inner = angleCenter - PI / teethCount * baseWidth;
      float angle2_inner = angleCenter + PI / teethCount * baseWidth;

      float angle1_outer = angleCenter - PI / teethCount * tipWidth;
      float angle2_outer = angleCenter + PI / teethCount * tipWidth;

      int x1_inner = cx + innerR * cos(angle1_inner);
      int y1_inner = cy + innerR * sin(angle1_inner);
      int x2_inner = cx + innerR * cos(angle2_inner);
      int y2_inner = cy + innerR * sin(angle2_inner);
      int x1_outer = cx + outerR * cos(angle1_outer);
      int y1_outer = cy + outerR * sin(angle1_outer);
      int x2_outer = cx + outerR * cos(angle2_outer);
      int y2_outer = cy + outerR * sin(angle2_outer);

      tft.fillTriangle(x1_inner, y1_inner, x1_outer, y1_outer, x2_outer, y2_outer, color);
      tft.fillTriangle(x1_inner, y1_inner, x2_outer, y2_outer, x2_inner, y2_inner, color);
    }

    tft.fillCircle(cx, cy, innerR, color);
    tft.fillCircle(cx, cy, centerR, C_BG);
    tft.drawCircle(cx, cy, centerR, color);
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
  
  void drawRoscaPage1(int sel){
    tft.fillScreen(C_BG);

    // Header: "< THREAD SELECTION" — Orbitron 8pt, white, top-left
    tft.setFont(&Orbitron_Medium8pt7b);
    tft.setTextColor(C_WHITE);
    tft.setCursor(28, 33);
    tft.print("< THREAD SELECTION");
    tft.setFont(nullptr);

    // Button-Menu icon — top-right
    tft.drawRGBBitmap(BTN_MENU_X, BTN_MENU_Y, btn_menu_off, BTN_MENU_W, BTN_MENU_H);

    // Thread pitch grid page 1 — 2×3 (ROSCA_P1=6 items, all slots filled)
    const int COLS = 2;
    const int TAB_W = 110, TAB_H = 20, TAB_GAP_X = 20, TAB_GAP_Y = 15;
    const int GRID_X = 40, GRID_Y = 65;

    tft.setFont(&Orbitron_Medium8pt7b);
    for(int i = 0; i < ROSCA_P1; i++){
      int col = i % COLS, row = i / COLS;
      int x = GRID_X + col * (TAB_W + TAB_GAP_X);
      int y = GRID_Y + row * (TAB_H + TAB_GAP_Y);
      bool isSel = (i == sel);
      tft.fillRect(x, y, TAB_W, TAB_H, isSel ? C_ACCENT : C_BTN);
      tft.setTextColor(isSel ? C_WHITE : C_MUTED);
      char buf[8]; snprintf(buf, sizeof(buf), "%.2f", roscas[i]);
      int16_t x1, y1; uint16_t tw, th;
      tft.getTextBounds(buf, 0, 0, &x1, &y1, &tw, &th);
      tft.setCursor(x + (TAB_W - (int)tw) / 2 - x1, y + (TAB_H - (int)th) / 2 - y1);
      tft.print(buf);
    }
    tft.setFont(nullptr);

    // Pagination "1 / 2" — left caret OFF (no prev), right caret ON (next available)
    tft.drawRGBBitmap(17, 182, solid_left_off, SOLID_W, SOLID_H);
    tft.setFont(&Orbitron_Medium8pt7b);
    tft.setTextColor(C_WHITE);
    tft.setCursor(44, 199);
    tft.print("1 / 2");
    int afterPag = tft.getCursorX();
    tft.setFont(nullptr);
    tft.drawRGBBitmap(afterPag + 4, 182, solid_on, SOLID_W, SOLID_H);

    // Button-Back and Button-Ok — bottom-right
    tft.drawRGBBitmap(222, 182, btn_back_off, BTN_BACK_W, BTN_BACK_H);
    tft.drawRGBBitmap(262, 182, btn_ok_off, BTN_OK_W, BTN_OK_H);
  }

  void drawRoscaPage2(int sel){
    tft.fillScreen(C_BG);

    // Header: "< THREAD SELECTION" — Orbitron 8pt, white, top-left
    tft.setFont(&Orbitron_Medium8pt7b);
    tft.setTextColor(C_WHITE);
    tft.setCursor(28, 33);
    tft.print("< THREAD SELECTION");
    tft.setFont(nullptr);

    // Button-Menu icon — top-right
    tft.drawRGBBitmap(BTN_MENU_X, BTN_MENU_Y, btn_menu_off, BTN_MENU_W, BTN_MENU_H);

    // Thread pitch grid page 2 — 2×4 (7 items: indices ROSCA_P1..NUM_ROS-1, last slot empty)
    // Slightly tighter gap to fit 4 rows above the button bar
    const int COLS = 2;
    const int TAB_W = 110, TAB_H = 20, TAB_GAP_X = 20, TAB_GAP_Y = 12;
    const int GRID_X = 40, GRID_Y = 55;
    const int count = NUM_ROS - ROSCA_P1;  // 7

    tft.setFont(&Orbitron_Medium8pt7b);
    for(int i = 0; i < count; i++){
      int idx = ROSCA_P1 + i;
      int col = i % COLS, row = i / COLS;
      int x = GRID_X + col * (TAB_W + TAB_GAP_X);
      int y = GRID_Y + row * (TAB_H + TAB_GAP_Y);
      bool isSel = (idx == sel);
      tft.fillRect(x, y, TAB_W, TAB_H, isSel ? C_ACCENT : C_BTN);
      tft.setTextColor(isSel ? C_WHITE : C_MUTED);
      char buf[8]; snprintf(buf, sizeof(buf), "%.2f", roscas[idx]);
      int16_t x1, y1; uint16_t tw, th;
      tft.getTextBounds(buf, 0, 0, &x1, &y1, &tw, &th);
      tft.setCursor(x + (TAB_W - (int)tw) / 2 - x1, y + (TAB_H - (int)th) / 2 - y1);
      tft.print(buf);
    }
    tft.setFont(nullptr);

    // Pagination "2 / 2" — left caret ON (prev available), right caret OFF (no next)
    tft.drawRGBBitmap(17, 182, solid_left_on, SOLID_W, SOLID_H);
    tft.setFont(&Orbitron_Medium8pt7b);
    tft.setTextColor(C_WHITE);
    tft.setCursor(44, 199);
    tft.print("2 / 2");
    int afterPag = tft.getCursorX();
    tft.setFont(nullptr);
    tft.drawRGBBitmap(afterPag + 4, 182, solid_off, SOLID_W, SOLID_H);

    // Button-Back and Button-Ok — bottom-right
    tft.drawRGBBitmap(222, 182, btn_back_off, BTN_BACK_W, BTN_BACK_H);
    tft.drawRGBBitmap(262, 182, btn_ok_off, BTN_OK_W, BTN_OK_H);
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

// ===== Variables compartidas Core0 ↔ Core1 =====
volatile AppState g_servoState  = AppState::SPLASH;
volatile int      g_avanceSel   = 0;
volatile int      g_roscaSel    = 0;
volatile uint8_t  g_leadPitch   = 3;
volatile bool     g_cwDirection = true;
volatile bool     g_servoActive = false;

// ===== Control servo Core1 — variables privadas =====
static int32_t servoLastE_c1       = 0;
static int32_t servoAccumulator_c1 = 0;

void IRAM_ATTR updateServoCore1() {
  // Lectura directa de variables volatile — segura para tipos <= 32 bits en ESP32
  // No se usa portENTER_CRITICAL en el loop caliente para evitar contención con Core0
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

  float F = (st == AppState::AVANCE_MODE) ? avances[avSel] : roscas[roSel];
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
  ets_delay_us(2);  // Setup time DIR→PUL (Lichuan A6) — no deshabilita interrupciones
  for (int32_t i = 0; i < pulsesToSend; i++) {
    digitalWrite(PUL_PIN, HIGH);
    ets_delay_us(3);  // Ancho de pulso mínimo Lichuan A6
    digitalWrite(PUL_PIN, LOW);
    ets_delay_us(3);  // Separación entre pulsos
  }
}

class Application {
private:
  Adafruit_ILI9341 tft;
  DisplayManager disp;
  Button bUp, bDown, bSel, bBack;
  AppState state;
  AppState prevState;
  int menuSel, configMenuSel, ratioDigitSel, avanceSel, roscaSel;
  int menuOverlaySel;
  float ang, lastAng;
  uint32_t tRPM, tANG, splashStart;
  int lastRPM;
  Config config;

public:
  Application()
  : tft(TFT_CS, TFT_DC, TFT_RST), disp(tft),
    bUp(BTN_UP), bDown(BTN_DOWN), bSel(BTN_SELECT), bBack(BTN_BACK),
    state(AppState::SPLASH), prevState(AppState::MAIN_MENU),
    menuSel(0), configMenuSel(0), ratioDigitSel(0),
    avanceSel(0), roscaSel(0), menuOverlaySel(0),
    ang(0.0f), lastAng(-1000.0f),
    tRPM(0), tANG(0), splashStart(0), lastRPM(0) {}

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
    pinMode(BRAKE_PIN, OUTPUT);  // GPIO reservado para freno (sin uso por ahora)
    pinMode(ENC_Z, INPUT_PULLUP);  // GPIO reservado para señal Z del encoder (sin uso por ahora)
    digitalWrite(PUL_PIN, LOW);
    digitalWrite(BRAKE_PIN, LOW);  // Estado por defecto
    // touch.begin();         // Disabled: Wokwi does not support XPT2046
    // touch.setRotation(1);
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
      // handleTouch();  // Disabled: Wokwi does not support XPT2046
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
    // handleTouch();  // Disabled: Wokwi does not support XPT2046
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
          state = AppState::ROSCA_SELECT_P1;
          disp.drawRoscaPage1(roscaSel);
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
    else if(state==AppState::ROSCA_SELECT_P1){
      if(bUp.read()==ButtonEvent::PRESSED){
        roscaSel--;
        if(roscaSel < 0){
          roscaSel = NUM_ROS - 1;
          state = AppState::ROSCA_SELECT_P2;
          disp.drawRoscaPage2(roscaSel);
        } else {
          disp.drawRoscaPage1(roscaSel);
        }
      }
      if(bDown.read()==ButtonEvent::PRESSED){
        roscaSel++;
        if(roscaSel >= ROSCA_P1){
          state = AppState::ROSCA_SELECT_P2;
          disp.drawRoscaPage2(roscaSel);
        } else {
          disp.drawRoscaPage1(roscaSel);
        }
      }
      if(bSel.read()==ButtonEvent::PRESSED){
        state = AppState::ROSCADO_MODE;
        g_servoState  = AppState::ROSCADO_MODE;
        g_avanceSel   = avanceSel;
        g_roscaSel    = roscaSel;
        g_leadPitch   = config.leadScrewPitch;
        g_cwDirection = config.cwDirection;
        g_servoActive = true;
        disp.modeScreen("< THREAD MODE", roscas[roscaSel], "Pitch");
      }
    }
    else if(state==AppState::ROSCA_SELECT_P2){
      if(bUp.read()==ButtonEvent::PRESSED){
        roscaSel--;
        if(roscaSel < ROSCA_P1){
          roscaSel = ROSCA_P1 - 1;
          state = AppState::ROSCA_SELECT_P1;
          disp.drawRoscaPage1(roscaSel);
        } else {
          disp.drawRoscaPage2(roscaSel);
        }
      }
      if(bDown.read()==ButtonEvent::PRESSED){
        roscaSel++;
        if(roscaSel >= NUM_ROS){
          roscaSel = 0;
          state = AppState::ROSCA_SELECT_P1;
          disp.drawRoscaPage1(roscaSel);
        } else {
          disp.drawRoscaPage2(roscaSel);
        }
      }
      if(bSel.read()==ButtonEvent::PRESSED){
        state = AppState::ROSCADO_MODE;
        g_servoState  = AppState::ROSCADO_MODE;
        g_avanceSel   = avanceSel;
        g_roscaSel    = roscaSel;
        g_leadPitch   = config.leadScrewPitch;
        g_cwDirection = config.cwDirection;
        g_servoActive = true;
        disp.modeScreen("< THREAD MODE", roscas[roscaSel], "Pitch");
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
      if(state==AppState::ROSCA_SELECT_P2){
        state = AppState::ROSCA_SELECT_P1;
        if(roscaSel >= ROSCA_P1) roscaSel = 0;
        disp.drawRoscaPage1(roscaSel);
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

  // handleTouch() — Disabled for Wokwi (XPT2046 not supported by simulator)
  // Re-enable by uncommenting this method and the handleTouch() calls in run()
  // when deploying to real hardware.
  void handleTouch(){
    // if(!touch.touched()) return;
    // TS_Point p = touch.getPoint();
    // int x = map(p.x, TS_MAXX, TS_MINX, 0, 319);
    // int y = map(p.y, TS_MAXY, TS_MINY, 0, 239);
    // x = constrain(x,0,319);
    // y = constrain(y,0,239);

    // // Splash: any touch skips to main
    // if(state == AppState::SPLASH) {
    //   state = AppState::MAIN_MENU;
    //   disp.hud();
    //   disp.mainMenu(menuSel);
    //   delay(200);
    //   return;
    // }

    // // Menu overlay touch handling
    // if(state == AppState::MENU_OVERLAY) {
    //   // Back button (circle at ~155,192 r=15)
    //   if(x>=140 && x<=170 && y>=177 && y<=207) {
    //     state = prevState;
    //     disp.hud();
    //     disp.mainMenu(menuSel);
    //     delay(200);
    //     return;
    //   }
    //   // Options
    //   for(int i = 0; i < 3; i++) {
    //     int oy = 65 + i * 40;
    //     if(x>=130 && x<=300 && y>=oy && y<=oy+28) {
    //       menuOverlaySel = i;
    //       disp.drawMenuOverlay(menuOverlaySel);
    //       delay(200);
    //       if(i == 0) {
    //         state = AppState::CONFIG_MENU;
    //         disp.configMenu(configMenuSel);
    //       }
    //       // INTERFACE and INFO: reserved for future implementation
    //       return;
    //     }
    //   }
    //   return;
    // }

    // // Button-Menu touch (top-right, all screens except splash/overlay)
    // // Figma: Button-Menu at approx x=295-319, y=8-32
    // if(x>=290 && x<=319 && y>=8 && y<=32) {
    //   prevState = state;
    //   menuOverlaySel = 0;
    //   state = AppState::MENU_OVERLAY;
    //   disp.drawMenuOverlay(menuOverlaySel);
    //   delay(200);
    //   return;
    // }

    // // Back arrow touch (top-right area, sub-screens)
    // if(state!=AppState::MAIN_MENU){
    //   if(x>=280 && x<=319 && y<=40){
    //     if(state==AppState::CONFIG_DIRECTION || state==AppState::CONFIG_RATIO){
    //       state = AppState::CONFIG_MENU;
    //       disp.configMenu(configMenuSel);
    //     } else {
    //       state = AppState::MAIN_MENU;
    //       disp.hud();
    //       disp.mainMenu(menuSel);
    //     }
    //     delay(200);
    //     return;
    //   }
    // }

    // if(state==AppState::MAIN_MENU){
    //   if(y>=80 && y<133){
    //     state = AppState::AVANCE_SELECT;
    //     disp.drawAvanceSelect(avanceSel);
    //     delay(200);
    //     return;
    //   }
    //   if(y>=133 && y<186){
    //     state = AppState::ROSCA_SELECT_P1;
    //     disp.drawRoscaPage1(roscaSel);
    //     delay(200);
    //     return;
    //   }
    //   if(y>=186){
    //     state = AppState::DIVISOR_MODE;
    //     disp.hud();
    //     disp.divisorBase();
    //     delay(200);
    //     return;
    //   }
    // }

    // if(state==AppState::CONFIG_MENU){
    //   if(y>=80 && y<160){
    //     state = AppState::CONFIG_DIRECTION;
    //     disp.configDirection(config.cwDirection);
    //     delay(200);
    //   } else if(y>=160){
    //     state = AppState::CONFIG_RATIO;
    //     ratioDigitSel = 0;
    //     disp.configRatio(config.leadScrewPitch, ratioDigitSel);
    //     delay(200);
    //   }
    //   return;
    // }

    // if(state==AppState::CONFIG_DIRECTION){
    //   if(y>=90 && y<=140){
    //     if(x>=40 && x<=150){
    //       config.cwDirection = true;
    //       disp.configDirection(config.cwDirection);
    //       delay(200);
    //     } else if(x>=170 && x<=280){
    //       config.cwDirection = false;
    //       disp.configDirection(config.cwDirection);
    //       delay(200);
    //     }
    //   }
    //   if(x>=CONF_X && x<=CONF_X+CONF_W && y>=170 && y<=194){
    //     config.save();
    //     state = AppState::CONFIG_MENU;
    //     disp.configMenu(configMenuSel);
    //     delay(200);
    //   }
    //   return;
    // }

    // if(state==AppState::CONFIG_RATIO){
    //   if(x>=CONF_X && x<=CONF_X+CONF_W && y>=195 && y<=219){
    //     config.save();
    //     state = AppState::CONFIG_MENU;
    //     disp.configMenu(configMenuSel);
    //     delay(200);
    //   }
    //   return;
    // }

    // if(state==AppState::AVANCE_SELECT){
    //   for(int i=0;i<NUM_AV;i++){
    //     int row = i/2, col = i%2;
    //     int bx = (col==0)? BTN_LEFT_X : BTN_RIGHT_X;
    //     int by = GRID_START_Y + row*(BTN_H + BTN_GAP_Y);
    //     if(x>=bx && x<=bx+BTN_W && y>=by && y<=by+BTN_H){
    //       avanceSel = i;
    //       disp.drawAvanceSelect(avanceSel);
    //       delay(200);
    //       return;
    //     }
    //   }
    //   if(x>=CONF_X && x<=CONF_X+CONF_W && y>=CONF_Y && y<=CONF_Y+CONF_H){
    //     char buf[16];
    //     snprintf(buf,sizeof(buf),"%.2f mm", avances[avanceSel]);
    //     state = AppState::AVANCE_MODE;
    //     disp.hud();
    //     disp.modeScreen("FEED MODE", buf);
    //     delay(200);
    //   }
    //   return;
    // }

    // if(state==AppState::ROSCA_SELECT_P1){
    //   for(int i=0;i<ROSCA_P1;i++){
    //     int row = i/2, col = i%2;
    //     int bx = (col==0)? BTN_LEFT_X : BTN_RIGHT_X;
    //     int by = GRID_START_Y + row*(BTN_H + BTN_GAP_Y);
    //     if(x>=bx && x<=bx+BTN_W && y>=by && y<=by+BTN_H){
    //       roscaSel = i;
    //       disp.drawRoscaPage1(roscaSel);
    //       delay(200);
    //       return;
    //     }
    //   }
    //   if(x>=260 && x<=310 && y>=ARROW_Y && y<=ARROW_Y+ARROW_H){
    //     state = AppState::ROSCA_SELECT_P2;
    //     disp.drawRoscaPage2(roscaSel);
    //     delay(200);
    //     return;
    //   }
    //   if(x>=CONF_X && x<=CONF_X+CONF_W && y>=CONF_Y && y<=CONF_Y+CONF_H){
    //     char buf[16];
    //     snprintf(buf,sizeof(buf),"%.2f pitch", roscas[roscaSel]);
    //     state = AppState::ROSCADO_MODE;
    //     disp.hud();
    //     disp.modeScreen("THREAD MODE", buf);
    //     delay(200);
    //   }
    //   return;
    // }

    // if(state==AppState::ROSCA_SELECT_P2){
    //   int base = ROSCA_P1, count = NUM_ROS - ROSCA_P1;
    //   for(int i=0;i<count;i++){
    //     int idx = base + i, row = i/2, col = i%2;
    //     int bx = (col==0)? BTN_LEFT_X : BTN_RIGHT_X;
    //     int by = GRID_START_Y + row*(BTN_H + BTN_GAP_Y);
    //     if(x>=bx && x<=bx+BTN_W && y>=by && y<=by+BTN_H){
    //       roscaSel = idx;
    //       disp.drawRoscaPage2(roscaSel);
    //       delay(200);
    //       return;
    //     }
    //   }
    //   if(x>=10 && x<=60 && y>=ARROW_Y && y<=ARROW_Y+ARROW_H){
    //     state = AppState::ROSCA_SELECT_P1;
    //     disp.drawRoscaPage1(roscaSel);
    //     delay(200);
    //     return;
    //   }
    //   if(x>=CONF_X && x<=CONF_X+CONF_W && y>=CONF_Y && y<=CONF_Y+CONF_H){
    //     char buf[16];
    //     snprintf(buf,sizeof(buf),"%.2f pitch", roscas[roscaSel]);
    //     state = AppState::ROSCADO_MODE;
    //     disp.hud();
    //     disp.modeScreen("THREAD MODE", buf);
    //     delay(200);
    //   }
    //   return;
    // }

    // if(state==AppState::DIVISOR_MODE){
    //   if(x>=60 && x<=260 && y>=140 && y<=190){
    //     encCount = 0;
    //     lastAng = -1000.0f;
    //     delay(200);
    //   }
    // }
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

// ===== Core1: tarea FreeRTOS dedicada al servo (ESP32) =====
void servoTask(void* pvParameters) {
  servoLastE_c1 = encCount;  // Sincronizar posición inicial
  for (;;) {
    updateServoCore1();
    vTaskDelay(1);  // Ceder al scheduler — evita WDT y permite que Core1 atienda otras tareas
  }
}

void setup(){
  Serial.begin(115200);
  app.begin();
  // Lanzar tarea servo en Core1 con prioridad alta (24) y stack 2KB
  xTaskCreatePinnedToCore(
    servoTask,    // función de la tarea
    "ServoTask",  // nombre (debug)
    4096,         // stack en bytes (float + arrays PROGMEM requieren stack suficiente)
    nullptr,      // parámetro
    24,           // prioridad (0=mín, 25=máx en ESP32)
    nullptr,      // handle (no necesario)
    1             // Core1
  );
}