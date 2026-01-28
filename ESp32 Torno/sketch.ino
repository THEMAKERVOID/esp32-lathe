#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
// #include <XPT2046_Touchscreen.h>
#include <math.h>

// ==================== PINES ====================
#define TFT_CS    10
#define TFT_DC    11
#define TFT_RST   12
#define TFT_SCK   15
#define TFT_MOSI  13
#define TFT_MISO  14

// #define TOUCH_CS  16
// #define TOUCH_IRQ 255  // no usado (busy polling)

#define BTN_UP      4
#define BTN_DOWN    5
#define BTN_SELECT  6
#define BTN_BACK    7

// Encoder husillo
#define ENC_A 20
#define ENC_B 21

// Servo Lichuan A6 (PUL/DIR)
#define PUL_PIN 8
#define DIR_PIN 9

// XPT2046_Touchscreen touch(TOUCH_CS, TOUCH_IRQ);

// ==================== CONSTANTES ====================
constexpr int   QUAD = 1440;          // pulsos por vuelta (cuadratura)
constexpr float LEAD = 3.0f;          // paso husillo carro (mm/vuelta)
constexpr uint16_t UPDATE_RPM   = 200;
constexpr uint16_t UPDATE_ANGLE = 50;
constexpr uint16_t DEBOUNCE_DELAY = 200;

// Calibración touch
// constexpr int TS_MINX = 395;
// constexpr int TS_MAXX = 3846;
// constexpr int TS_MINY = 412;
// constexpr int TS_MAXY = 3739;

// ==================== AVANCES ====================
const float avances[] = {0.07f, 0.10f, 0.13f, 0.18f, 0.25f};
const int   NUM_AV   = sizeof(avances)/sizeof(avances[0]);

// ==================== ROSCAS (13 valores) ====================
const float roscas[] = {
  0.40f, 0.50f, 0.70f, 0.80f, 1.00f, 1.25f,
  1.50f, 1.75f, 2.00f, 2.50f, 3.00f, 3.50f, 4.00f
};
const int   NUM_ROS  = 13;

constexpr int ROSCA_P1 = 6;   // primera página: 6 roscas, segunda: 7

// ==================== LAYOUT BOTONES FULLSCREEN ====================
constexpr int GRID_START_Y = 60;
constexpr int BTN_H        = 26;
constexpr int BTN_GAP_Y    = 6;
constexpr int BTN_W        = 130;

constexpr int BTN_LEFT_X   = 15;
constexpr int BTN_RIGHT_X  = 175;

// Flechas y botón OK alineados en Y
constexpr int ARROW_Y = 190;
constexpr int ARROW_W = 50;
constexpr int ARROW_H = 24;

// Botón OK (2/3 del ancho anterior)
constexpr int CONF_W = 133;
constexpr int CONF_H = 24;
constexpr int CONF_X = (320 - CONF_W) / 2;
constexpr int CONF_Y = 190;

// ==================== ESTADOS ====================
enum class AppState : uint8_t {
  MAIN_MENU,
  AVANCE_MODE,
  AVANCE_SELECT,
  ROSCADO_MODE,
  ROSCA_SELECT_P1,
  ROSCA_SELECT_P2,
  DIVISOR_MODE
};

enum class ButtonEvent : uint8_t {
  NONE,
  PRESSED
};

// ==================== BOTÓN HW ====================
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

// ==================== ENCODER ISR ====================
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

// ==================== SERVO ====================
inline void servoForward(){
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(PUL_PIN, HIGH);
  digitalWrite(PUL_PIN, LOW);
}
inline void servoBackward(){
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(PUL_PIN, HIGH);
  digitalWrite(PUL_PIN, LOW);
}

// ==================== DISPLAY ====================
class DisplayManager {
private:
  Adafruit_ILI9341& tft;

public:
  DisplayManager(Adafruit_ILI9341& d): tft(d) {}

  void begin(){
    SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI, TFT_CS);
    tft.begin();
    tft.setRotation(1);
    tft.fillScreen(ILI9341_BLACK);
  }

  // HUD de RPM (parte superior)
  void hud(){
    tft.fillRect(0,0,320,80, ILI9341_BLACK);
    tft.drawRect(0,0,320,80, ILI9341_DARKGREY);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_CYAN);
    tft.setCursor(10,10);
    tft.print("RPM");
  }

  void drawRPM(int rpm){
    tft.fillRect(118,16,84,34, ILI9341_BLACK);
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
      int w  = head - abs(y)*head/hh;
      int xs = cx - body - w;
      tft.drawFastHLine(xs, cy+y, w, c);
    }
    tft.fillRect(cx-body, cy-hh/2, body, hh, c);
  }

  // ===== Menú principal (AVANCE / ROSCADO / DIVISOR) =====
  void mainMenu(int sel){
    tft.fillRect(0,80,320,160, ILI9341_BLACK);

    const char* it[3] = {"AVANCE","ROSCADO","DIVISOR"};
    int y = 80;
    int h[3] = {53,53,54};

    for(int i=0;i<3;i++){
      uint16_t bg = (i==sel)? ILI9341_GREEN : ILI9341_BLACK;
      uint16_t tc = (i==sel)? ILI9341_BLACK : ILI9341_WHITE;

      tft.fillRect(0,y,320,h[i], bg);
      tft.drawRect(0,y,320,h[i], ILI9341_WHITE);

      tft.setTextSize(3);
      tft.setTextColor(tc);
      int tw = (int)strlen(it[i])*18;
      tft.setCursor((320 - tw)/2, y + h[i]/2 - 12);
      tft.print(it[i]);

      y += h[i];
    }
  }

  // ===== Pantalla de modo (AVANCE / ROSCADO) mostrando selección =====
  void modeScreen(const char* titulo, const char* selText){
    tft.fillRect(0,80,320,160, ILI9341_BLACK);
    drawBack();

    tft.setTextSize(3);
    tft.setTextColor(ILI9341_WHITE);

    int tw = (int)strlen(titulo)*18;
    tft.setCursor((320 - tw)/2, 100);
    tft.print(titulo);

    // Valor seleccionado debajo
    tft.setTextSize(3);
    int tw2 = (int)strlen(selText)*18;
    tft.setCursor((320 - tw2)/2, 100 + 28);
    tft.print(selText);
  }

  // ===== Divisor =====
  void divisorBase(){
    tft.fillRect(0,80,320,160, ILI9341_BLACK);
    drawBack();

    tft.setTextSize(3);
    tft.setTextColor(ILI9341_WHITE);
    const char* t = "DIVISOR";
    int tw = (int)strlen(t)*18;
    tft.setCursor((320 - tw)/2, 100);
    tft.print(t);

    tft.drawRect(60,140,200,50, ILI9341_WHITE);
    tft.fillRect(61,141,198,48, ILI9341_BLACK);

    tft.setTextSize(2);
    tft.setTextColor(ILI9341_CYAN);
    const char* h = "Puesta a cero";
    int th = (int)strlen(h)*12;
    tft.setCursor((320 - th)/2, 200);
    tft.print(h);
  }

  void divisorAngle(float ang){
    tft.fillRect(66,145,188,28, ILI9341_BLACK);
    char b[16];
    snprintf(b,sizeof(b),"%.2f DRG",ang);
    tft.setTextSize(3);
    tft.setTextColor(ILI9341_WHITE);
    int tw = (int)strlen(b)*18;
    tft.setCursor(60 + (200 - tw)/2, 150);
    tft.print(b);
  }

  // ===== Comunes submenús fullscreen =====
  void drawSelectTitle(const char* title){
    tft.fillScreen(ILI9341_BLACK);
    drawBack();
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_CYAN);
    int tw = (int)strlen(title)*12;
    tft.setCursor((320 - tw)/2, 12);
    tft.print(title);
  }

  void drawButton(int x, int y, float v, bool sel){
    uint16_t bg = sel ? ILI9341_GREEN : ILI9341_DARKGREY;
    tft.fillRect(x, y, BTN_W, BTN_H, bg);
    tft.drawRect(x, y, BTN_W, BTN_H, ILI9341_WHITE);

    char b[16];
    snprintf(b,sizeof(b),"%.2f", v);
    int tw = (int)strlen(b)*12;
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(x + (BTN_W - tw)/2, y + 6);
    tft.print(b);
  }

  // ===== ROSCA página 1 (6 valores) =====
  void drawRoscaPage1(int sel){
    drawSelectTitle("Roscas (1/2)");

    for(int i=0;i<ROSCA_P1;i++){
      int row = i/2;
      int col = i%2;
      int x = (col==0)? BTN_LEFT_X : BTN_RIGHT_X;
      int y = GRID_START_Y + row*(BTN_H + BTN_GAP_Y);
      drawButton(x, y, roscas[i], sel==i);
    }

    // Flecha →
    tft.fillRect(260, ARROW_Y, ARROW_W, ARROW_H, ILI9341_BLUE);
    tft.drawRect(260, ARROW_Y, ARROW_W, ARROW_H, ILI9341_WHITE);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(278, ARROW_Y + 6);
    tft.print(">");

    // OK
    tft.fillRect(CONF_X, CONF_Y, CONF_W, CONF_H, ILI9341_BLUE);
    tft.drawRect(CONF_X, CONF_Y, CONF_W, CONF_H, ILI9341_WHITE);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(CONF_X + (CONF_W/2 - 12), CONF_Y + 6);
    tft.print("OK");
  }

  // ===== ROSCA página 2 (7 valores) =====
  void drawRoscaPage2(int sel){
    drawSelectTitle("Roscas (2/2)");

    int base  = ROSCA_P1;
    int count = NUM_ROS - ROSCA_P1;

    for(int i=0;i<count;i++){
      int idx = base + i;
      int row = i/2;
      int col = i%2;
      int x = (col==0)? BTN_LEFT_X : BTN_RIGHT_X;
      int y = GRID_START_Y + row*(BTN_H + BTN_GAP_Y);
      drawButton(x, y, roscas[idx], sel==idx);
    }

    // Flecha ←
    tft.fillRect(10, ARROW_Y, ARROW_W, ARROW_H, ILI9341_BLUE);
    tft.drawRect(10, ARROW_Y, ARROW_W, ARROW_H, ILI9341_WHITE);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(30, ARROW_Y + 6);
    tft.print("<");

    // OK
    tft.fillRect(CONF_X, CONF_Y, CONF_W, CONF_H, ILI9341_BLUE);
    tft.drawRect(CONF_X, CONF_Y, CONF_W, CONF_H, ILI9341_WHITE);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(CONF_X + (CONF_W/2 - 12), CONF_Y + 6);
    tft.print("OK");
  }

  // ===== AVANCE fullscreen =====
  void drawAvanceSelect(int sel){
    tft.fillScreen(ILI9341_BLACK);
    drawBack();

    tft.setTextSize(2);
    tft.setTextColor(ILI9341_CYAN);
    const char* t = "Seleccion avance";
    int tw = (int)strlen(t)*12;
    tft.setCursor((320 - tw)/2, 12);
    tft.print(t);

    for(int i=0;i<NUM_AV;i++){
      int row = i/2;
      int col = i%2;
      int x = (col==0)? BTN_LEFT_X : BTN_RIGHT_X;
      int y = GRID_START_Y + row*(BTN_H + BTN_GAP_Y);
      drawButton(x, y, avances[i], sel==i);
    }

    // OK
    tft.fillRect(CONF_X, CONF_Y, CONF_W, CONF_H, ILI9341_BLUE);
    tft.drawRect(CONF_X, CONF_Y, CONF_W, CONF_H, ILI9341_WHITE);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(CONF_X + (CONF_W/2 - 12), CONF_Y + 6);
    tft.print("OK");
  }
};

// ==================== APLICACIÓN ====================
class Application {
private:
  Adafruit_ILI9341 tft;
  DisplayManager   disp;

  Button bUp, bDown, bSel, bBack;

  AppState state;
  int menuSel;

  int   avanceSel;
  int   roscaSel;

  float ang;
  float lastAng;
  uint32_t tRPM, tANG;
  int lastRPM;

public:
  Application()
  : tft(TFT_CS, TFT_DC, TFT_RST),
    disp(tft),
    bUp(BTN_UP), bDown(BTN_DOWN),
    bSel(BTN_SELECT), bBack(BTN_BACK),
    state(AppState::MAIN_MENU),
    menuSel(0),
    avanceSel(0),
    roscaSel(0),
    ang(0.0f), lastAng(-1000.0f),
    tRPM(0), tANG(0),
    lastRPM(0)
  {}

  void begin(){
    disp.begin();
    disp.hud();
    disp.mainMenu(menuSel);

    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    attachInterrupt(ENC_A, isrA, CHANGE);
    attachInterrupt(ENC_B, isrB, CHANGE);

    pinMode(PUL_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(PUL_PIN, LOW);

    // touch.begin();
    // touch.setRotation(1);
  }

  void run(){
    uint32_t now = millis();

    // RPM visibles en estos estados:
    if(state==AppState::MAIN_MENU ||
       state==AppState::AVANCE_MODE ||
       state==AppState::ROSCADO_MODE ||
       state==AppState::DIVISOR_MODE)
    {
      if(now - tRPM >= UPDATE_RPM){
        updateRPM();
        disp.drawRPM(lastRPM);
        tRPM = now;
      }
    }

    // Ángulo solo en divisor
    if(state==AppState::DIVISOR_MODE){
      if(now - tANG >= UPDATE_ANGLE){
        updateAngle();
        tANG = now;
      }
    }

    // Servo sincronizado con husillo
    if(state==AppState::AVANCE_MODE || state==AppState::ROSCADO_MODE){
      updateServo();
    }

    handleButtons();
    // handleTouch();  // Touch comentado
    updateScreen();
  }

private:
  void updateRPM(){
    static int32_t last = 0;
    int32_t cur  = encCount;
    int32_t diff = cur - last;
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

  void updateServo(){
    static int32_t lastE = 0;
    int32_t cur  = encCount;
    int32_t diff = cur - lastE;
    if(diff==0) return;
    lastE = cur;

    float F = (state==AppState::AVANCE_MODE) ? avances[avanceSel]
                                             : roscas[roscaSel];

    float factor = F / LEAD;
    float pulses = diff * factor;

    if(pulses > 0){
      int p = int(pulses);
      for(int i=0;i<p;i++) servoForward();
    } else if(pulses < 0){
      int p = -int(pulses);
      for(int i=0;i<p;i++) servoBackward();
    }
  }

  void handleButtons(){
    // ===== MENÚ PRINCIPAL =====
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
        if(menuSel==0){
          state = AppState::AVANCE_SELECT;
          disp.drawAvanceSelect(avanceSel);
        }else if(menuSel==1){
          state = AppState::ROSCA_SELECT_P1;
          disp.drawRoscaPage1(roscaSel);
        }else{
          state = AppState::DIVISOR_MODE;
          disp.hud();
          disp.divisorBase();
        }
      }
    }

    // ===== AVANCE SELECT =====
    else if(state==AppState::AVANCE_SELECT){
      if(bUp.read()==ButtonEvent::PRESSED){
        avanceSel = (avanceSel + NUM_AV - 1) % NUM_AV;
        disp.drawAvanceSelect(avanceSel);
      }
      if(bDown.read()==ButtonEvent::PRESSED){
        avanceSel = (avanceSel + 1) % NUM_AV;
        disp.drawAvanceSelect(avanceSel);
      }
      if(bSel.read()==ButtonEvent::PRESSED){
        char buf[16];
        snprintf(buf,sizeof(buf),"%.2f mm", avances[avanceSel]);
        state = AppState::AVANCE_MODE;
        disp.hud();
        disp.modeScreen("MODO AVANCE", buf);
      }
    }

    // ===== ROSCA SELECT P1 =====
    else if(state==AppState::ROSCA_SELECT_P1){
      if(bUp.read()==ButtonEvent::PRESSED){
        roscaSel--;
        if(roscaSel < 0){
          // Cambiar a página 2 (última rosca)
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
          // Cambiar a página 2 (primera rosca de P2)
          state = AppState::ROSCA_SELECT_P2;
          disp.drawRoscaPage2(roscaSel);
        } else {
          disp.drawRoscaPage1(roscaSel);
        }
      }
      if(bSel.read()==ButtonEvent::PRESSED){
        // Confirmar selección
        char buf[16];
        snprintf(buf,sizeof(buf),"%.2f paso", roscas[roscaSel]);
        state = AppState::ROSCADO_MODE;
        disp.hud();
        disp.modeScreen("MODO ROSCADO", buf);
      }
    }

    // ===== ROSCA SELECT P2 =====
    else if(state==AppState::ROSCA_SELECT_P2){
      if(bUp.read()==ButtonEvent::PRESSED){
        roscaSel--;
        if(roscaSel < ROSCA_P1){
          // Volver a página 1 (última rosca de P1)
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
          // Volver a página 1 (primera rosca)
          roscaSel = 0;
          state = AppState::ROSCA_SELECT_P1;
          disp.drawRoscaPage1(roscaSel);
        } else {
          disp.drawRoscaPage2(roscaSel);
        }
      }
      if(bSel.read()==ButtonEvent::PRESSED){
        // Confirmar selección
        char buf[16];
        snprintf(buf,sizeof(buf),"%.2f paso", roscas[roscaSel]);
        state = AppState::ROSCADO_MODE;
        disp.hud();
        disp.modeScreen("MODO ROSCADO", buf);
      }
    }

    // ===== DIVISOR MODE =====
    else if(state==AppState::DIVISOR_MODE){
      if(bSel.read()==ButtonEvent::PRESSED){
        encCount = 0;
        lastAng  = -1000.0f;
      }
    }

    // ===== BOTÓN BACK =====
    if(bBack.read()==ButtonEvent::PRESSED && state!=AppState::MAIN_MENU){
      if(state==AppState::ROSCA_SELECT_P2){
        state = AppState::ROSCA_SELECT_P1;
        if(roscaSel >= ROSCA_P1) roscaSel = 0;
        disp.drawRoscaPage1(roscaSel);
      }
      else {
        state = AppState::MAIN_MENU;
        disp.hud();
        disp.mainMenu(menuSel);
      }
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

// ==================== SETUP / LOOP ====================
Application app;

void setup(){
  Serial.begin(115200);
  app.begin();
}

void loop(){
  app.run();
}