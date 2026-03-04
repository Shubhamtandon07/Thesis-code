#include <TFT_eSPI.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

/* ========================= ORIGINAL SPEEDOMETER CODE (UNTOUCHED) ========================= */
TFT_eSPI tft = TFT_eSPI();
#define LOOP_PERIOD 50 

#define TFT_DARKGRAY   0x4208
#define TFT_MEDIUMGRAY 0x7BEF
#define TFT_LIGHTGRAY  0xAD55
#define TFT_DARKBLUE   0x0010
#define TFT_LIGHTBLUE  0x5D1C
#define TFT_DARKGREEN  0x03E0
#define TFT_ORANGE     0xFD20
#define TFT_MAROON     0x7800

int speed = 0;          
int gear = 1;           
int old_speed = -1;     
int old_gear = -1;      
uint32_t updateTime = 0; 

#define START_ANGLE -200
#define END_ANGLE    20

// ---------- NeoPixel (LED ring) ----------
#define LED_PIN  7
#define NUM_LEDS 8
Adafruit_NeoPixel leds(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
bool ledWasRed = false;   // for detecting rising edge of red

// ---------- HX711 Strain Gauge ----------
#define HX711_DOUT 38
#define HX711_SCK  2

long strain_zero_value  = 0;
long strain_threshold   = 70000;
float sensitivity_factor = 1.0;
bool calibrated = false;

const int numReadings = 8;  
long readings[numReadings];
int readIndex = 0;
long total = 0;
long average = 0;

// ---------- TFT backlight control (optional) ----------
// If your display backlight is connected to a GPIO, set the pin here.
// For example, many ESP32 boards use GPIO 21 or 32 for backlight.
// Uncomment and set the correct pin if needed.
// #define TFT_BL 21
// void setBacklightBrightness(uint8_t brightness) { analogWrite(TFT_BL, brightness); }

// forward declarations
void drawUI();
void drawZone(int start_value, int end_value, uint16_t start_color, uint16_t end_color);
uint16_t interpolateColor(uint16_t color1, uint16_t color2, float ratio);
void updateNeedle(int spd);
void drawNeedle(int angle, uint16_t color);
void displaySpeed(int spd);
void displayGear(int g);
long readHX711();
void setAllLEDs(uint32_t color);
void calibrateStrainGauge();
void checkSerialCommands();

/* ========================= MODE MANAGEMENT (3 MODES) ========================= */
enum UIMode : uint8_t { MODE_SPEEDO=0, MODE_TORQUE=1, MODE_RPM=2 };
volatile UIMode g_mode = MODE_SPEEDO;
volatile int    g_detent_index = 0;   // 0..2
volatile bool   g_detent_changed = false;

/* ========================= RPM STATE ========================= */
static const int RPM_MAX = 8000;
int rpm = 0;
int old_rpm = -1;
static int rpm_old_angleDeg = START_ANGLE;

// Forward decls (Torque + RPM UIs)
void torque_drawBase();
void torque_drawDynamic();
void rpm_drawBase();
void rpm_drawDynamic();
void rpm_displayValue(int rpmVal);
void rpm_drawNeedleVal(int rpmVal, uint16_t color);

void switchMode(UIMode m) {
  if (g_mode == m) return;
  g_mode = m;
  if (g_mode == MODE_SPEEDO) {
    drawUI();
    displayGear(gear);
    displaySpeed(speed);
    Serial.println("MODE: SPEED");
  } else if (g_mode == MODE_TORQUE) {
    torque_drawBase();
    torque_drawDynamic();
    Serial.println("MODE: TORQUE");
  } else {
    rpm_drawBase();
    rpm_drawDynamic();
    Serial.println("MODE: RPM");
  }
}

/* ========================= HFX HAPTICS (with press vibration) ========================= */
#if defined(ARDUINO_ARCH_ESP32)
  #include "driver/ledc.h"
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
#endif

// Encoder MT6701 SSI pins
#define ENC_CS   14
#define ENC_CLK  13
#define ENC_DO   37

// TMC6300 pins
#define MOTOR_UH_PIN 25
#define MOTOR_UL_PIN 33
#define MOTOR_VH_PIN 26
#define MOTOR_VL_PIN 32
#define MOTOR_WH_PIN 27
#define MOTOR_WL_PIN 21

#if defined(ARDUINO_ARCH_ESP32)
  static const ledc_mode_t      HFX_MODE  = LEDC_LOW_SPEED_MODE;
  static const ledc_timer_t     HFX_TIMER = LEDC_TIMER_0;
  static const ledc_timer_bit_t HFX_BITS  = LEDC_TIMER_10_BIT;
  static const uint32_t         HFX_FREQ  = 20000;
  static const ledc_channel_t   HFX_CH[6] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2,
    LEDC_CHANNEL_3, LEDC_CHANNEL_4, LEDC_CHANNEL_5
  };
  static const int HFX_GPIO[6] = {
    MOTOR_UH_PIN, MOTOR_UL_PIN, MOTOR_VH_PIN,
    MOTOR_VL_PIN, MOTOR_WH_PIN, MOTOR_WL_PIN
  };
#endif

static const int   POLE_PAIRS            = 7;
static const int   DETENT_COUNT          = 3;
static const float DETENT_SPACING_DEG    = 120.0f;
static const float HYSTERESIS_DEG        = 25.0f;
static const float KP                    = 1.5f;
static const float KD                    = 0.004f;
static const float STATIC_FRICTION       = 0.20f;
static const float TORQUE_LIMIT          = 0.80f;
static const float ELECTRICAL_OFFSET_DEG = 90.0f;

volatile bool g_haptic_pulse = false;   // request a vibration pulse

static float g_zero_offset    = 0.0f;
static bool  g_zero_calibrated = false;

// utils
static inline float wrapDeg(float a) { while (a >= 360.0f) a -= 360.0f; while (a < 0.0f) a += 360.0f; return a; }
static inline float angDelta(float a, float b) { float d = wrapDeg(a) - wrapDeg(b); if (d > 180.0f) d -= 360.0f; if (d < -180.0f) d += 360.0f; return d; }
static inline float getCalibratedAngle(float rawDeg) { return wrapDeg(rawDeg - g_zero_offset); }

static bool mt6701_read_deg(float& degOut) {
  digitalWrite(ENC_CS, LOW);
  delayMicroseconds(1);
  uint16_t raw = 0;
  for (int i = 0; i < 16; ++i) {
    digitalWrite(ENC_CLK, HIGH); delayMicroseconds(1);
    raw = (uint16_t)((raw << 1) | (digitalRead(ENC_DO) ? 1 : 0));
    digitalWrite(ENC_CLK, LOW);  delayMicroseconds(1);
  }
  digitalWrite(ENC_CS, HIGH);
  degOut = ((raw & 0x3FFF) * 360.0f) / 16384.0f;
  return true;
}

#if defined(ARDUINO_ARCH_ESP32)
static void hfx_ledc_init() {
  ledc_timer_config_t tcfg = {};
  tcfg.speed_mode       = HFX_MODE;
  tcfg.timer_num        = HFX_TIMER;
  tcfg.duty_resolution  = HFX_BITS;
  tcfg.freq_hz          = HFX_FREQ;
  tcfg.clk_cfg          = LEDC_AUTO_CLK;
  ledc_timer_config(&tcfg);
  for (int i = 0; i < 6; i++) {
    ledc_channel_config_t cc = {};
    cc.gpio_num   = HFX_GPIO[i];
    cc.speed_mode = HFX_MODE;
    cc.channel    = HFX_CH[i];
    cc.intr_type  = LEDC_INTR_DISABLE;
    cc.timer_sel  = HFX_TIMER;
    cc.duty       = 0;
    cc.hpoint     = 0;
    ledc_channel_config(&cc);
  }
}
static inline void pwmWrite_LEDC(int idx, int duty) {
  int maxd = (1 << HFX_BITS) - 1;
  duty = constrain(duty, 0, maxd);
  ledc_set_duty(HFX_MODE, HFX_CH[idx], duty);
  ledc_update_duty(HFX_MODE, HFX_CH[idx]);
}
#endif

static void setPhases(float da, float db, float dc) {
#if defined(ARDUINO_ARCH_ESP32)
  int maxd = (1 << HFX_BITS) - 1;
  pwmWrite_LEDC(0, (int)(constrain(da, 0.0f, 1.0f) * maxd)); pwmWrite_LEDC(1, 0);
  pwmWrite_LEDC(2, (int)(constrain(db, 0.0f, 1.0f) * maxd)); pwmWrite_LEDC(3, 0);
  pwmWrite_LEDC(4, (int)(constrain(dc, 0.0f, 1.0f) * maxd)); pwmWrite_LEDC(5, 0);
#else
  (void)da; (void)db; (void)dc;
#endif
}

static void driveBLDC(float mechDeg, float tq) {
#if !defined(ARDUINO_ARCH_ESP32)
  (void)mechDeg; (void)tq; return;
#endif
  const float elecDeg  = mechDeg * POLE_PAIRS + ELECTRICAL_OFFSET_DEG;
  const float phaseDeg = elecDeg + (tq >= 0 ? 90.0f : -90.0f);
  const float amp      = fabsf(tq);
  setPhases(
    0.5f + 0.5f * amp * sinf(phaseDeg * DEG_TO_RAD),
    0.5f + 0.5f * amp * sinf((phaseDeg - 120.0f) * DEG_TO_RAD),
    0.5f + 0.5f * amp * sinf((phaseDeg + 120.0f) * DEG_TO_RAD)
  );
}

// 80ms alternating-torque buzz – gives a sharp "click" feeling
static void haptic_vibrate(float mechDeg) {
  for (int s = 0; s < 8; s++) {
    driveBLDC(mechDeg, (s % 2 == 0) ? 0.65f : -0.65f);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

#if defined(ARDUINO_ARCH_ESP32)
static void hfx_task(void*) {
  vTaskDelay(pdMS_TO_TICKS(150));

  float firstDeg = 0;
  if (!mt6701_read_deg(firstDeg)) {
    Serial.println("[HFX] Encoder read failed. Haptics disabled.");
    vTaskDelete(NULL); return;
  }
  float lastDeg = firstDeg;
  int   lastPublished = -1;
  int   stable_count = 0;
  const int STABLE_THRESH = 5;
  uint32_t lastMicros = micros();

  while (true) {
    // Haptic pulse from strain gauge press takes priority
    if (g_haptic_pulse) {
      g_haptic_pulse = false;
      float cur = 0;
      mt6701_read_deg(cur);
      haptic_vibrate(g_zero_calibrated ? getCalibratedAngle(cur) : cur);
      lastMicros = micros();
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    uint32_t now = micros();
    float dt = (now - lastMicros) / 1e6f;
    if (dt < 0.0008f) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }
    lastMicros = now;

    float rawAng = 0;
    if (!mt6701_read_deg(rawAng)) { setPhases(0.5f, 0.5f, 0.5f); vTaskDelay(pdMS_TO_TICKS(2)); continue; }

    float angDeg = g_zero_calibrated ? getCalibratedAngle(rawAng) : rawAng;
    float vel    = angDelta(angDeg, lastDeg) / dt;
    lastDeg = angDeg;

    int nearest = (int)floor((angDeg + (DETENT_SPACING_DEG / 2.0f)) / DETENT_SPACING_DEG) % DETENT_COUNT;
    float target = wrapDeg(nearest * DETENT_SPACING_DEG);
    float err = angDelta(angDeg, target);

    const float slip = (DETENT_SPACING_DEG * 0.5f) + HYSTERESIS_DEG;
    if (err >  slip) { nearest = (nearest + 1) % DETENT_COUNT; target = wrapDeg(nearest * DETENT_SPACING_DEG); err = angDelta(angDeg, target); }
    if (err < -slip) { nearest = (nearest + (DETENT_COUNT - 1)) % DETENT_COUNT; target = wrapDeg(nearest * DETENT_SPACING_DEG); err = angDelta(angDeg, target); }

    float tq = (KP * err) - (KD * vel);
    if (fabsf(err) > slip * 0.8f) tq += (err > 0 ? 0.1f : -0.1f);
    if (fabsf(err) < 12.0f) {
      float stick = STATIC_FRICTION * (1.0f - (fabsf(err) / 12.0f));
      tq += (err >= 0 ? stick : -stick);
    }
    tq = constrain(tq, -TORQUE_LIMIT, TORQUE_LIMIT);
    driveBLDC(angDeg, tq);

    if (nearest != lastPublished) {
      if (++stable_count >= STABLE_THRESH) {
        lastPublished = nearest;
        g_detent_index = nearest;
        g_detent_changed = true;
        stable_count = 0;
      }
    } else {
      stable_count = 0;
    }
  }
}
#endif

void HFX_init() {
  pinMode(ENC_CS,  OUTPUT); digitalWrite(ENC_CS, HIGH);
  pinMode(ENC_CLK, OUTPUT); digitalWrite(ENC_CLK, LOW);
  pinMode(ENC_DO,  INPUT);
#if defined(ARDUINO_ARCH_ESP32)
  hfx_ledc_init();
  xTaskCreatePinnedToCore(hfx_task, "hfx_detents", 4096, nullptr, 1, nullptr, 0);
#endif
}
void HFX_update() {}

void calibrateZero() {
  float raw;
  if (mt6701_read_deg(raw)) {
    g_zero_offset = raw;
    g_zero_calibrated = true;
    Serial.print("Zero offset set to: "); Serial.println(g_zero_offset);
  } else {
    Serial.println("Zero calibration failed - encoder read error");
  }
}

/* ========================= TORQUE UI (adjusted diameter, thicker needle) ========================= */
static const int   CX = 120, CY = 120;
static const int   R_IN = 68,  R_OUT = 98;
static const float TSTART = -210.0f, TEND = 30.0f;

float TORQUE_MAX_NM = 50.0f;
float torqueNm = 0.0f;
float peakNm = 0.0f;
float oldTorqueNm = -1.0f;

static inline float mapf(float v, float a0, float a1, float b0, float b1) { return b0 + (v - a0) * (b1 - b0) / (a1 - a0); }
static inline float d2r(float d) { return d * DEG_TO_RAD; }
uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) { return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3); }

void torque_drawBase() {
  tft.fillScreen(TFT_DARKBLUE);
  tft.fillCircle(CX, CY, R_OUT + 12, TFT_DARKGRAY);
  tft.fillCircle(CX, CY, R_OUT + 6,  TFT_BLACK);

  const int steps = 120;
  for (int i = 0; i < steps; i++) {
    float t0 = (float)i / steps, t1 = (float)(i + 1) / steps;
    float a0 = d2r(mapf(t0, 0, 1, TSTART, TEND));
    float a1 = d2r(mapf(t1, 0, 1, TSTART, TEND));
    int x00 = CX + cosf(a0) * R_IN,  y00 = CY + sinf(a0) * R_IN;
    int x01 = CX + cosf(a1) * R_IN,  y01 = CY + sinf(a1) * R_IN;
    int x10 = CX + cosf(a0) * R_OUT, y10 = CY + sinf(a0) * R_OUT;
    int x11 = CX + cosf(a1) * R_OUT, y11 = CY + sinf(a1) * R_OUT;
    uint16_t col = rgb565(35, 40, 50);
    tft.fillTriangle(x00, y00, x01, y01, x11, y11, col);
    tft.fillTriangle(x00, y00, x10, y10, x11, y11, col);
  }

  const int majors = 10;
  for (int i = 0; i <= majors; i++) {
    float f = (float)i / majors;
    float ang = d2r(mapf(f, 0, 1, TSTART, TEND));
    int r0 = R_OUT + 2, r1 = r0 + (i % 2 == 0 ? 10 : 6);
    int x0 = CX + cosf(ang) * r0, y0 = CY + sinf(ang) * r0;
    int x1 = CX + cosf(ang) * r1, y1 = CY + sinf(ang) * r1;
    tft.drawLine(x0, y0, x1, y1, (i % 2 == 0) ? TFT_WHITE : TFT_LIGHTGRAY);
    if (i % 2 == 0) {
      int lx = CX + cosf(ang) * (R_OUT + 20);
      int ly = CY + sinf(ang) * (R_OUT + 20);
      tft.setTextColor(TFT_WHITE, TFT_DARKBLUE);
      tft.drawCentreString(String((int)(f * TORQUE_MAX_NM)), lx, ly, 2);
    }
  }
  tft.setTextColor(TFT_LIGHTGRAY, TFT_DARKBLUE);
  tft.drawCentreString("N·m", CX, CY - 62, 2);
}

void torque_drawProgress(float nm) {
  float frac = TORQUE_MAX_NM > 0 ? nm / TORQUE_MAX_NM : 0;
  if (frac < 0) frac = 0;
  if (frac > 1) frac = 1;
  const int steps = 120;
  int n = (int)(steps * frac);
  for (int i = 0; i < n; i++) {
    float t0 = (float)i / steps, t1 = (float)(i + 1) / steps;
    float a0 = d2r(mapf(t0, 0, 1, TSTART, TEND));
    float a1 = d2r(mapf(t1, 0, 1, TSTART, TEND));
    float mix = (float)i / (steps - 1);
    uint16_t col = (mix < 0.5f) ?
      rgb565((uint8_t)(0 + (int)((240 - 0) * (mix * 2))),
             (uint8_t)(180 + (int)((200 - 180) * (mix * 2))),
             40) :
      rgb565((uint8_t)(240 + (int)((210 - 240) * ((mix - 0.5f) * 2))),
             (uint8_t)(200 + (int)((40 - 200) * ((mix - 0.5f) * 2))),
             40);
    int x00 = CX + cosf(a0) * R_IN,  y00 = CY + sinf(a0) * R_IN;
    int x01 = CX + cosf(a1) * R_IN,  y01 = CY + sinf(a1) * R_IN;
    int x10 = CX + cosf(a0) * R_OUT, y10 = CY + sinf(a0) * R_OUT;
    int x11 = CX + cosf(a1) * R_OUT, y11 = CY + sinf(a1) * R_OUT;
    tft.fillTriangle(x00, y00, x01, y01, x11, y11, col);
    tft.fillTriangle(x00, y00, x10, y10, x11, y11, col);
  }
}

void torque_drawNeedle(float nm) {
  float f = TORQUE_MAX_NM > 0 ? nm / TORQUE_MAX_NM : 0;
  if (f < 0) f = 0;
  if (f > 1) f = 1;
  float angle = d2r(mapf(f, 0, 1, TSTART, TEND));
  int baseR = 27;
  int cx = CX, cy = CY;
  int xTip = cx + (int)(cosf(angle) * 80);
  int yTip = cy + (int)(sinf(angle) * 80);
  int xb1 = cx + (int)(cosf(angle + 2.5f) * baseR);
  int yb1 = cy + (int)(sinf(angle + 2.5f) * baseR);
  int xb2 = cx + (int)(cosf(angle - 2.5f) * baseR);
  int yb2 = cy + (int)(sinf(angle - 2.5f) * baseR);
  uint16_t color = (nm < TORQUE_MAX_NM * 0.4f) ? TFT_GREEN : (nm < TORQUE_MAX_NM * 0.75f) ? TFT_YELLOW : TFT_RED;
  tft.fillTriangle(xTip, yTip, xb1, yb1, xb2, yb2, color);
  tft.fillCircle(cx, cy, 5, TFT_DARKGRAY);
  tft.drawCircle(cx, cy, 5, TFT_LIGHTGRAY);
}

void torque_drawNumeric(float nm) {
  tft.fillCircle(CX, CY, 46, TFT_BLACK);
  tft.drawCircle(CX, CY, 46, TFT_LIGHTGRAY);
  char buf[16]; dtostrf(nm, 0, 1, buf);
  tft.setTextColor(TFT_LIGHTGRAY, TFT_BLACK);
  tft.drawCentreString(String(buf), CX + 2, CY + 2, 4);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawCentreString(String(buf), CX, CY, 4);
  char pbuf[16]; dtostrf(peakNm, 0, 1, pbuf);
  tft.setTextColor(TFT_LIGHTGRAY, TFT_DARKBLUE);
  tft.drawCentreString(String("PEAK ") + pbuf, CX, CY + 56, 2);
}

void torque_drawDynamic() {
  torque_drawBase();
  torque_drawProgress(torqueNm);
  torque_drawNeedle(torqueNm);
  torque_drawNumeric(torqueNm);
}

/* ========================= RPM UI (larger diameter) ========================= */
void rpm_drawBase() {
  tft.fillScreen(TFT_DARKBLUE);
  tft.fillCircle(120, 120, 115, TFT_DARKGRAY);
  tft.fillCircle(120, 120, 105, TFT_BLACK);

  for (int i = 0; i < 10; i++) tft.drawCircle(120, 120, 115 - i, tft.color565(40 + i * 5, 40 + i * 5, 40 + i * 5));
  for (int i = 0; i < 5; i++)  tft.drawCircle(120, 120, 105 - i, tft.color565(20 + i * 5, 20 + i * 5, 20 + i * 5));

  for (int i = 0; i <= RPM_MAX; i += 500) {
    float angle = map(i, 0, RPM_MAX, START_ANGLE, END_ANGLE) * DEG_TO_RAD;
    int tick_length = (i % 1000 == 0) ? 15 : 8;
    uint16_t tick_color = (i % 1000 == 0) ? TFT_LIGHTGRAY : TFT_MEDIUMGRAY;

    int x_start = cos(angle) * 90 + 120;
    int y_start = sin(angle) * 90 + 120;
    int x_end   = cos(angle) * (90 + tick_length) + 120;
    int y_end   = sin(angle) * (90 + tick_length) + 120;
    tft.drawLine(x_start, y_start, x_end, y_end, tick_color);

    if (i % 1000 == 0) {
      int lx = cos(angle) * 75 + 120;
      int ly = sin(angle) * 75 + 120;
      tft.setTextColor(TFT_MEDIUMGRAY, TFT_BLACK);
      tft.drawCentreString(String(i / 1000), lx, ly, 2);
    }
  }

  tft.setTextColor(TFT_MEDIUMGRAY, TFT_BLACK);
  tft.drawCentreString("x1000", 120, 62, 2);
  tft.setTextColor(TFT_LIGHTGRAY, TFT_BLACK);
  tft.drawCentreString("RPM", 120, 78, 2);

  rpm_old_angleDeg = START_ANGLE;
}

void rpm_drawNeedleVal(int rpmVal, uint16_t color) {
  int angleDeg = map(rpmVal, 0, RPM_MAX, START_ANGLE, END_ANGLE);
  float rad = angleDeg * DEG_TO_RAD;
  int cx = 120, cy = 120;

  int x_tip   = cos(rad) * 80 + cx;
  int y_tip   = sin(rad) * 80 + cy;
  int x_base1 = cos(rad + 2.8) * 15 + cx;
  int y_base1 = sin(rad + 2.8) * 15 + cy;
  int x_base2 = cos(rad - 2.8) * 15 + cx;
  int y_base2 = sin(rad - 2.8) * 15 + cy;

  tft.fillTriangle(x_tip, y_tip, x_base1, y_base1, x_base2, y_base2, color);
  tft.fillCircle(cx, cy, 5, TFT_DARKGRAY);
  tft.drawCircle(cx, cy, 5, TFT_LIGHTGRAY);

  rpm_old_angleDeg = angleDeg;
}

void rpm_displayValue(int rpmVal) {
  tft.fillRect(70, 105, 100, 30, TFT_BLACK);
  tft.setTextColor(TFT_MEDIUMGRAY, TFT_BLACK);
  tft.drawCentreString(String(rpmVal), 122, 107, 4);
  tft.setTextColor(TFT_LIGHTGRAY, TFT_BLACK);
  tft.drawCentreString(String(rpmVal), 120, 105, 4);
}

void rpm_drawDynamic() {
  int prevRpm = map(rpm_old_angleDeg, START_ANGLE, END_ANGLE, 0, RPM_MAX);
  rpm_drawNeedleVal(prevRpm, TFT_BLACK);

  uint16_t col = (rpm < 3000) ? TFT_GREEN : (rpm < 6000) ? TFT_YELLOW : TFT_RED;
  rpm_drawNeedleVal(rpm, col);
  rpm_displayValue(rpm);
}

/* ========================= SETUP / LOOP ========================= */
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Starting...");

  // Initialize TFT first
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  Serial.println("TFT initialized");

  // Optional TFT backlight control
  #ifdef TFT_BL
    pinMode(TFT_BL, OUTPUT);
    analogWrite(TFT_BL, 255);  // full brightness
  #endif

  // NeoPixels
  leds.begin();
  leds.setBrightness(255);  // Maximum brightness
  leds.clear();
  leds.show();
  Serial.println("NeoPixels initialized");

  // HX711
  pinMode(HX711_SCK, OUTPUT);
  pinMode(HX711_DOUT, INPUT);
  digitalWrite(HX711_SCK, LOW);
  for (int i = 0; i < numReadings; i++) readings[i] = 0;

  // Haptic motor
  HFX_init();
  calibrateStrainGauge();

  // Draw initial UI
  drawUI();
  displayGear(gear);
  displaySpeed(speed);
  Serial.println("MODE: SPEED");
}

void loop() {
  HFX_update();

  // ---------- Serial commands ----------
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    String up = line; up.trim(); up.toUpperCase();

    if (up == "CHANGE") { switchMode((UIMode)((g_mode + 1) % 3)); }
    else if (up == "TORQUE") { switchMode(MODE_TORQUE); }
    else if (up == "SPEED" || up == "SPEEDO") { switchMode(MODE_SPEEDO); }
    else if (up == "RPM") { switchMode(MODE_RPM); }

    else if (up.startsWith("S ")) {
      int v = up.substring(2).toInt();
      v = constrain(v, 0, 200);
      speed = v;
      if (speed == 0) gear = 0;
      else if (speed <= 20) gear = 1;
      else if (speed <= 40) gear = 2;
      else if (speed <= 80) gear = 3;
      else if (speed <= 120) gear = 4;
      else if (speed <= 160) gear = 5;
      else gear = 6;
    }
    else if (up.startsWith("Q ")) {
      float nm = up.substring(2).toFloat();
      nm = constrain(nm, 0.0f, TORQUE_MAX_NM);
      torqueNm = nm;
      if (torqueNm > peakNm) peakNm = torqueNm;
      oldTorqueNm = -1.0f;
    }
    else if (up.startsWith("R ")) {
      int v = up.substring(2).toInt();
      rpm = constrain(v, 0, RPM_MAX);
      old_rpm = -1;
    }
    else if (up.startsWith("M ")) {
      float m = up.substring(2).toFloat();
      if (m > 1 && m <= 200) {
        TORQUE_MAX_NM = m;
        if (peakNm > TORQUE_MAX_NM) peakNm = TORQUE_MAX_NM;
        oldTorqueNm = -1.0f;
      }
    }
    else {
      int v = 0;
      if (up.startsWith("SPEED:")) v = up.substring(6).toInt();
      else v = line.toInt();
      v = constrain(v, 0, 200);
      speed = v;
      if (speed == 0) gear = 0;
      else if (speed <= 20) gear = 1;
      else if (speed <= 40) gear = 2;
      else if (speed <= 80) gear = 3;
      else if (speed <= 120) gear = 4;
      else if (speed <= 160) gear = 5;
      else gear = 6;
    }
  }

  // ---------- TFT update at cadence ----------
  if (millis() >= updateTime) {
    updateTime = millis() + LOOP_PERIOD;
    if (g_mode == MODE_SPEEDO) {
      if (speed != old_speed) {
        updateNeedle(speed);
        displaySpeed(speed);
        old_speed = speed;
      }
      if (gear != old_gear) {
        displayGear(gear);
        old_gear = gear;
      }
    } else if (g_mode == MODE_TORQUE) {
      if (fabsf(torqueNm - oldTorqueNm) > 0.02f) {
        torque_drawDynamic();
        oldTorqueNm = torqueNm;
      }
    } else {
      if (rpm != old_rpm) {
        rpm_drawDynamic();
        old_rpm = rpm;
      }
    }
  }

  // ---------- HX711 + LED + haptic ----------
  long raw_value = readHX711();
  total -= readings[readIndex];
  readings[readIndex] = raw_value;
  total += readings[readIndex];
  readIndex = (readIndex + 1) % numReadings;
  average = total / numReadings;
  long strain_difference = labs(average - strain_zero_value) * sensitivity_factor;

  // LED logic + haptic on rising edge of red
  bool ledIsRed = (strain_difference > strain_threshold);
  if (ledIsRed) {
    setAllLEDs(leds.Color(255, 0, 0));
    if (!ledWasRed) {
      g_haptic_pulse = true;                // trigger vibration
      Serial.println("[HAPTIC] Press triggered");
    }
  } else {
    setAllLEDs(leds.Color(0, 255, 0));      // max green
  }
  ledWasRed = ledIsRed;

  // ---------- Debug output (every 500ms) ----------
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 500) {
    lastDebug = millis();
    Serial.print("Spd:"); Serial.print(speed);
    Serial.print(" Tq:"); Serial.print(torqueNm, 1);
    Serial.print(" RPM:"); Serial.print(rpm);
    Serial.print(" Str:"); Serial.print(strain_difference);
    Serial.print("/"); Serial.println(strain_threshold);
  }

  // Single-char commands
  checkSerialCommands();

  // Mode change from detent
  if (g_detent_changed) {
    g_detent_changed = false;
    switchMode((UIMode)(g_detent_index % 3));
  }

  delay(50);
}

/* ========================= ORIGINAL UI FUNCTIONS (modified diameters and needle) ========================= */
void drawUI() {
  tft.fillScreen(TFT_DARKBLUE);
  tft.fillCircle(120, 120, 112, TFT_DARKGRAY);
  tft.fillCircle(120, 120, 102, TFT_BLACK);
  for (int i = 0; i < 10; i++) tft.drawCircle(120, 120, 112 - i, tft.color565(40 + i * 5, 40 + i * 5, 40 + i * 5));
  for (int i = 0; i < 5; i++)  tft.drawCircle(120, 120, 102 - i, tft.color565(20 + i * 5, 20 + i * 5, 20 + i * 5));
  for (int i = 0; i <= 200; i += 10) {
    float angle = map(i, 0, 200, START_ANGLE, END_ANGLE) * DEG_TO_RAD;
    int tick_length = (i % 20 == 0) ? 15 : 8;
    uint16_t tick_color = (i % 40 == 0) ? TFT_WHITE : TFT_LIGHTGRAY;
    int x_start = cos(angle) * 88 + 120;
    int y_start = sin(angle) * 88 + 120;
    int x_end = cos(angle) * (88 + tick_length) + 120;
    int y_end = sin(angle) * (88 + tick_length) + 120;
    tft.drawLine(x_start, y_start, x_end, y_end, tick_color);
    if (i % 40 == 0) {
      int lx = cos(angle) * 73 + 120;
      int ly = sin(angle) * 73 + 120;
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.drawCentreString(String(i), lx, ly, 2);
    }
  }
  drawZone(0, 80, TFT_GREEN, TFT_DARKGREEN);
  drawZone(80, 140, TFT_YELLOW, TFT_ORANGE);
  drawZone(140, 200, TFT_RED, TFT_MAROON);
  tft.fillCircle(120, 120, 30, TFT_DARKGRAY);
  for (int i = 0; i < 5; i++) tft.drawCircle(120, 120, 30 - i, tft.color565(60 + i * 5, 60 + i * 5, 60 + i * 5));
  tft.setTextColor(TFT_LIGHTGRAY, TFT_BLACK);
  tft.drawCentreString("km/h", 122, 70, 2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawCentreString("km/h", 120, 68, 2);
  tft.fillRoundRect(80, 170, 80, 50, 10, TFT_DARKGRAY);
  tft.fillRoundRect(82, 172, 76, 46, 8, TFT_BLACK);
  tft.setTextColor(TFT_LIGHTGRAY, TFT_BLACK);
  tft.drawCentreString("GEAR", 120, 175, 2);
}

void drawZone(int start_value, int end_value, uint16_t start_color, uint16_t end_color) {
  int steps = end_value - start_value;
  for (int i = 0; i < steps; i++) {
    float ratio = (float)i / steps;
    uint16_t color = interpolateColor(start_color, end_color, ratio);
    float start_angle = map(start_value + i, 0, 200, START_ANGLE, END_ANGLE) * DEG_TO_RAD;
    float end_angle = map(start_value + i + 1, 0, 200, START_ANGLE, END_ANGLE) * DEG_TO_RAD;
    int x0 = cos(start_angle) * 88 + 120;
    int y0 = sin(start_angle) * 88 + 120;
    int x1 = cos(end_angle) * 88 + 120;
    int y1 = sin(end_angle) * 88 + 120;
    int x2 = cos(start_angle) * 102 + 120;
    int y2 = sin(start_angle) * 102 + 120;
    int x3 = cos(end_angle) * 102 + 120;
    int y3 = sin(end_angle) * 102 + 120;
    tft.fillTriangle(x0, y0, x1, y1, x3, y3, color);
    tft.fillTriangle(x0, y0, x2, y2, x3, y3, color);
  }
}

uint16_t interpolateColor(uint16_t color1, uint16_t color2, float ratio) {
  uint8_t r1 = (color1 >> 11) & 0x1F;
  uint8_t g1 = (color1 >> 5) & 0x3F;
  uint8_t b1 = color1 & 0x1F;
  uint8_t r2 = (color2 >> 11) & 0x1F;
  uint8_t g2 = (color2 >> 5) & 0x3F;
  uint8_t b2 = color2 & 0x1F;
  uint8_t r = r1 + (r2 - r1) * ratio;
  uint8_t g = g1 + (g2 - g1) * ratio;
  uint8_t b = b1 + (b2 - b1) * ratio;
  return (r << 11) | (g << 5) | b;
}

void updateNeedle(int spd) {
  static int old_angle = START_ANGLE;
  int new_angle = map(spd, 0, 200, START_ANGLE, END_ANGLE);
  uint16_t needle_color = (spd < 80) ? TFT_GREEN : (spd < 140) ? TFT_YELLOW : TFT_RED;
  drawNeedle(old_angle, TFT_BLACK);
  drawNeedle(new_angle, needle_color);
  old_angle = new_angle;
}

void drawNeedle(int angle, uint16_t color) {
  float rad = angle * DEG_TO_RAD;
  int cx = 120, cy = 120;
  int x_tip = cos(rad) * 80 + cx;
  int y_tip = sin(rad) * 80 + cy;
  int x_base1 = cos(rad + 2.5) * 22 + cx;
  int y_base1 = sin(rad + 2.5) * 22 + cy;
  int x_base2 = cos(rad - 2.5) * 22 + cx;
  int y_base2 = sin(rad - 2.5) * 22 + cy;
  tft.fillTriangle(x_tip, y_tip, x_base1, y_base1, x_base2, y_base2, color);
  tft.fillCircle(cx, cy, 5, TFT_DARKGRAY);
  tft.drawCircle(cx, cy, 5, TFT_LIGHTGRAY);
}

void displaySpeed(int spd) {
  tft.fillRect(85, 105, 70, 30, TFT_BLACK);
  tft.setTextColor(TFT_LIGHTGRAY, TFT_BLACK);
  tft.drawCentreString(String(spd), 122, 107, 4);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawCentreString(String(spd), 120, 105, 4);
}

void displayGear(int g) {
  tft.fillRect(95, 190, 50, 30, TFT_BLACK);
  tft.setTextColor(TFT_LIGHTGRAY, TFT_BLACK);
  tft.drawCentreString(g == 0 ? "N" : String(g), 97, 192, 6);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.drawCentreString(g == 0 ? "N" : String(g), 95, 190, 6);
}

// ---------------- HX711 ----------------
long readHX711() {
  while (digitalRead(HX711_DOUT) == HIGH);
  noInterrupts();
  long value = 0;
  for (int i = 0; i < 24; i++) {
    digitalWrite(HX711_SCK, HIGH);
    delayMicroseconds(1);
    value = (value << 1) | digitalRead(HX711_DOUT);
    digitalWrite(HX711_SCK, LOW);
    delayMicroseconds(1);
  }
  digitalWrite(HX711_SCK, HIGH);
  delayMicroseconds(1);
  digitalWrite(HX711_SCK, LOW);
  interrupts();
  if (value & 0x800000) value |= 0xFF000000;
  return value;
}

// ---------------- NeoPixel ----------------
void setAllLEDs(uint32_t color) {
  for (int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, color);
  leds.show();
}

void calibrateStrainGauge() {
  Serial.println("Calibrating strain gauge...");
  long sum = 0;
  int samples = 20;
  for (int i = 0; i < samples; i++) { sum += readHX711(); delay(50); }
  strain_zero_value = sum / samples;
  calibrated = true;
  Serial.print("Zero set to: ");
  Serial.println(strain_zero_value);
}

void checkSerialCommands() {
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case '+': sensitivity_factor += 0.5; break;
      case '-': sensitivity_factor = (sensitivity_factor > 0.5) ? sensitivity_factor - 0.5 : 0.5; break;
      case 't': strain_threshold = (strain_threshold > 500) ? strain_threshold - 500 : 500; break;
      case 'T': strain_threshold += 500; break;
      case 'c': calibrateStrainGauge(); break;
      case 'r': strain_threshold = 70000; sensitivity_factor = 1.0; peakNm = 0; break;
      case 's': speed = 0; gear = 0; old_speed = -1; old_gear = -1; Serial.println("[CMD] stop"); break;
      case 'g': Serial.println("[CMD] resume"); break;
      case 'x': switchMode((UIMode)((g_mode + 1) % 3)); break;
      case 'z': calibrateZero(); break;
      case 'v': g_haptic_pulse = true; Serial.println("[CMD] manual vibrate"); break;
    }
    Serial.print("Sensitivity: "); Serial.print(sensitivity_factor);
    Serial.print("  Threshold: "); Serial.println(strain_threshold);
  }
}
