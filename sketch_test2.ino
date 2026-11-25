#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <Wire.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// ========================================================
// SMART WATER BOTTLE - nRF52840 SuperMini
// OLED SSD1306 + LIS3DSH + RCWL-1670
// nRFMicro Core; USB Serial uses Adafruit_TinyUSB.h
// ========================================================

// --- Wiring: keep pins from the successful tests ---
#define SDA_PIN 2           // D2 = P0.17
#define SCL_PIN 3           // D3 = P0.20
#define TRIG_PIN 4          // D4 = P0.22
#define ECHO_PIN 5          // D5 = P0.24

#define OLED_WIDTH 128
#define OLED_HEIGHT 32      // Set 64 only for 128x64 OLED

// --- Bottle & sonar ---
#define BOTTLE_HEIGHT_CM 30.0f
#define SONAR_MIN_CM 3.0f
#define SONAR_MAX_CM 35.0f
#define SONAR_TIMEOUT_US 30000UL
#define SONAR_PING_GAP_MS 70UL
#define SONAR_BATCH_SIZE 5
#define SONAR_UPRIGHT_MS 900UL
#define SONAR_POST_SETTLE_MS 1500UL
#define VERIFY_TIMEOUT_MS 8500UL
#define MIN_WATER_DROP_CM 1.0f
#define POST_PAIR_MAX_DIFF_CM 0.8f
#define MAX_BASELINE_AGE_MS 15000UL

// --- LIS3DSH movement detection ---
#define ACC_SAMPLE_MS 80UL
#define CALIBRATION_SAMPLES 25
#define TILT_START_DEG 35.0f
#define UPRIGHT_DEG 20.0f
#define TILT_MIN_MS 700UL
#define TILT_MAX_MS 10000UL
#define ACC_DELTA_MAX_G 0.15f
#define ACC_MIN_G 0.80f
#define ACC_MAX_G 1.20f
#define MIN_STABLE_PERCENT 60UL

#define NOTIFY_DURATION_MS 2500UL
#define DISPLAY_REFRESH_MS 300UL

uint8_t oledAddr = 0;
uint8_t lisAddr = 0;
bool oledOK = false;
bool lisOK = false;
bool calibrated = false;

float refX = 0, refY = 0, refZ = 1;
float calX = 0, calY = 0, calZ = 0;
uint8_t calCount = 0;

float ax = 0, ay = 0, az = 0;
float lastX = 0, lastY = 0, lastZ = 0;
bool haveAccel = false;
float angleDeg = 0;
uint32_t uprightSinceMs = 0;

float waterDistance = -1;
int waterPercent = -1;
bool waterValid = false;
uint32_t lastWaterAtMs = 0;

uint16_t drinkCount = 0;
uint32_t notifyAtMs = 0;
bool notifyActive = false;

enum MotionState { READY, TILTING, VERIFYING, WAIT_UPRIGHT };
MotionState motionState = READY;
uint32_t tiltStartMs = 0;
uint32_t lastTiltedMs = 0;
uint16_t tiltedSamples = 0;
uint16_t stableSamples = 0;
float beforeDrinkDistance = -1;
uint32_t returnedAtMs = 0;
float firstAfterDistance = -1;

// Sonar: collect 5 independent readings with at least 70ms gap
float sonarReadings[SONAR_BATCH_SIZE];
uint8_t sonarAttempts = 0;
uint8_t sonarAccepted = 0;
uint32_t lastPingMs = 0;
uint32_t lastOledMs = 0;
uint32_t lastLogMs = 0;

// --------------------------------------------------------
// I2C and LIS3DSH
// --------------------------------------------------------
bool i2cPresent(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool i2cReadReg(uint8_t address, uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, (uint8_t)1) != 1) return false;
  value = Wire.read();
  return true;
}

bool lisWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(lisAddr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool initLIS3DSH() {
  const uint8_t candidates[] = {0x1E, 0x1D};
  for (uint8_t i = 0; i < 2; i++) {
    uint8_t id = 0;
    if (i2cReadReg(candidates[i], 0x0F, id) && id == 0x3F) {
      lisAddr = candidates[i];
      break;
    }
  }
  if (!lisAddr) return false;
  // +-2g, auto increment for 6-byte read, 100Hz / XYZ axes
  if (!lisWriteReg(0x24, 0x00)) return false;
  if (!lisWriteReg(0x25, 0x10)) return false;
  if (!lisWriteReg(0x20, 0x6F)) return false;
  delay(100);
  return true;
}

bool readAccel(float &x, float &y, float &z) {
  Wire.beginTransmission(lisAddr);
  Wire.write(0x28);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(lisAddr, (uint8_t)6) != 6) return false;

  uint8_t b[6];
  for (uint8_t i = 0; i < 6; i++) b[i] = Wire.read();
  int16_t rx = (int16_t)(((uint16_t)b[1] << 8) | b[0]);
  int16_t ry = (int16_t)(((uint16_t)b[3] << 8) | b[2]);
  int16_t rz = (int16_t)(((uint16_t)b[5] << 8) | b[4]);

  x = rx * 0.00006f;
  y = ry * 0.00006f;
  z = rz * 0.00006f;
  return true;
}

float magnitude3(float x, float y, float z) {
  return sqrtf(x*x + y*y + z*z);
}

float angleBetween(float x, float y, float z,
                   float bx, float by, float bz) {
  float m1 = magnitude3(x, y, z);
  float m2 = magnitude3(bx, by, bz);
  if (m1 < 0.01f || m2 < 0.01f) return 180.0f;
  float dot = (x*bx + y*by + z*bz) / (m1*m2);
  if (dot > 1.0f) dot = 1.0f;
  if (dot < -1.0f) dot = -1.0f;
  return acosf(dot) * 57.29578f;
}

// --------------------------------------------------------
// RCWL-1670: trigger/echo, batch median
// --------------------------------------------------------
float sonarOnceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(20);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long pulse = pulseIn(ECHO_PIN, HIGH, SONAR_TIMEOUT_US);
  if (pulse == 0) return -1.0f;
  float cm = pulse * 0.0343f * 0.5f;
  if (cm < SONAR_MIN_CM || cm > SONAR_MAX_CM) return -1.0f;
  return cm;
}

void resetSonarBatch() {
  sonarAttempts = 0;
  sonarAccepted = 0;
}

// Returns true only when a 5-ping batch finishes.
// resultCm = -1 when fewer than 3 valid samples.
bool getSonarMedian(float &resultCm) {
  uint32_t now = millis();
  if (now - lastPingMs < SONAR_PING_GAP_MS) return false;
  lastPingMs = now;

  float d = sonarOnceCm();
  sonarAttempts++;
  if (d >= 0.0f) sonarReadings[sonarAccepted++] = d;
  if (sonarAttempts < SONAR_BATCH_SIZE) return false;

  resultCm = -1.0f;
  if (sonarAccepted >= 3) {
    for (uint8_t i = 1; i < sonarAccepted; i++) {
      float v = sonarReadings[i];
      int j = i - 1;
      while (j >= 0 && sonarReadings[j] > v) {
        sonarReadings[j + 1] = sonarReadings[j];
        j--;
      }
      sonarReadings[j + 1] = v;
    }
    resultCm = sonarReadings[sonarAccepted / 2];
  }
  resetSonarBatch();
  return true;
}

int distanceToPercent(float cm) {
  float pct = (BOTTLE_HEIGHT_CM - cm) * 100.0f / BOTTLE_HEIGHT_CM;
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return (int)(pct + 0.5f);
}

void saveWaterLevel(float distance) {
  waterDistance = distance;
  waterPercent = distanceToPercent(distance);
  waterValid = true;
  lastWaterAtMs = millis();
}

void registerDrink(float dropCm) {
  drinkCount++;
  notifyActive = true;
  notifyAtMs = millis();
  Serial.print("[DRINK] #");
  Serial.print(drinkCount);
  Serial.print(" | drop=");
  Serial.print(dropCm, 2);
  Serial.print(" cm | approx=");
  Serial.print(dropCm * 100.0f / BOTTLE_HEIGHT_CM, 1);
  Serial.println("% of bottle capacity");
}

// --------------------------------------------------------
// Finite-state drink recognition
// --------------------------------------------------------
void updateMotion() {
  if (!lisOK) return;
  static uint32_t lastAccelMs = 0;
  uint32_t now = millis();
  if (now - lastAccelMs < ACC_SAMPLE_MS) return;
  lastAccelMs = now;

  float x, y, z;
  if (!readAccel(x, y, z)) {
    haveAccel = false;
    uprightSinceMs = 0;
    if (motionState == TILTING || motionState == VERIFYING)
      motionState = WAIT_UPRIGHT;
    return;
  }

  float m = magnitude3(x, y, z);
  float change = haveAccel ? magnitude3(x-lastX, y-lastY, z-lastZ) : 999.0f;
  ax = x; ay = y; az = z;
  lastX = x; lastY = y; lastZ = z;
  haveAccel = true;

  bool gravityOK = (m >= ACC_MIN_G && m <= ACC_MAX_G);
  bool stable = gravityOK && (change <= ACC_DELTA_MAX_G);

  // First ~2 seconds: calibrate upright reference. Keep bottle upright!
  if (!calibrated) {
    if (!stable) {
      calCount = 0;
      calX = calY = calZ = 0;
      return;
    }
    calX += x; calY += y; calZ += z;
    calCount++;
    if (calCount >= CALIBRATION_SAMPLES) {
      float norm = magnitude3(calX, calY, calZ);
      if (norm > 0.01f) {
        refX = calX/norm;
        refY = calY/norm;
        refZ = calZ/norm;
        calibrated = true;
        uprightSinceMs = now;
        Serial.println("[LIS] Upright calibration OK");
      }
    }
    return;
  }

  angleDeg = angleBetween(x, y, z, refX, refY, refZ);
  bool uprightNow = stable && angleDeg <= UPRIGHT_DEG;
  if (uprightNow) {
    if (uprightSinceMs == 0) uprightSinceMs = now;
  } else {
    uprightSinceMs = 0;
  }

  switch (motionState) {
    case READY:
      if (gravityOK && angleDeg >= TILT_START_DEG &&
          waterValid && now - lastWaterAtMs <= MAX_BASELINE_AGE_MS) {
        beforeDrinkDistance = waterDistance;
        tiltStartMs = lastTiltedMs = now;
        tiltedSamples = stableSamples = 0;
        motionState = TILTING;
        resetSonarBatch();
        Serial.println("[TILT] Begin");
      }
      break;

    case TILTING:
      if (gravityOK && angleDeg >= TILT_START_DEG) {
        lastTiltedMs = now;
        tiltedSamples++;
        if (stable) stableSamples++;
      }

      if (now - tiltStartMs > TILT_MAX_MS + 2000UL) {
        motionState = WAIT_UPRIGHT;
        Serial.println("[TILT] Too long, rejected");
        break;
      }

      if (uprightNow) {
        uint32_t tiltMs = lastTiltedMs - tiltStartMs;
        bool goodDuration = tiltMs >= TILT_MIN_MS && tiltMs <= TILT_MAX_MS;
        bool goodStability = tiltedSamples >= 4 &&
          ((uint32_t)stableSamples * 100UL >=
           (uint32_t)tiltedSamples * MIN_STABLE_PERCENT);

        Serial.print("[TILT] Duration=");
        Serial.print(tiltMs);
        Serial.print("ms stable=");
        Serial.print(stableSamples);
        Serial.print("/");
        Serial.println(tiltedSamples);

        if (goodDuration && goodStability) {
          motionState = VERIFYING;
          returnedAtMs = now;
          firstAfterDistance = -1;
          resetSonarBatch();
          Serial.println("[VERIFY] Waiting for calm water");
        } else {
          motionState = READY;
          Serial.println("[TILT] Not a drink gesture");
        }
      }
      break;

    case VERIFYING:
      // Reject if user tilts the bottle again before post-measurement.
      if (angleDeg >= TILT_START_DEG) {
        motionState = WAIT_UPRIGHT;
        resetSonarBatch();
        Serial.println("[VERIFY] Interrupted by another tilt");
      }
      break;

    case WAIT_UPRIGHT:
      if (uprightSinceMs != 0 && now - uprightSinceMs >= SONAR_UPRIGHT_MS) {
        motionState = READY;
        resetSonarBatch();
      }
      break;
  }
}

void updateWater() {
  uint32_t now = millis();
  bool isUpright = !lisOK ||
    (calibrated && uprightSinceMs != 0 &&
     now - uprightSinceMs >= SONAR_UPRIGHT_MS);

  // Normal level measurement while bottle is standing upright.
  if (motionState == READY) {
    if (!isUpright) { resetSonarBatch(); return; }
    float d;
    if (getSonarMedian(d) && d >= 0.0f) {
      saveWaterLevel(d);
    }
    return;
  }

  // Post-drink confirmation: two consistent sonar medians.
  if (motionState == VERIFYING) {
    if (now - returnedAtMs >= VERIFY_TIMEOUT_MS) {
      motionState = WAIT_UPRIGHT;
      firstAfterDistance = -1;
      resetSonarBatch();
      Serial.println("[VERIFY] Timeout - no drink counted");
      return;
    }
    bool calmEnough = calibrated && uprightSinceMs != 0 &&
      now - uprightSinceMs >= SONAR_POST_SETTLE_MS;
    if (!calmEnough) {
      firstAfterDistance = -1;
      resetSonarBatch();
      return;
    }

    float d;
    if (!getSonarMedian(d)) return;
    if (d < 0) return;

    if (firstAfterDistance < 0) {
      firstAfterDistance = d;
      return;
    }

    if (fabsf(d - firstAfterDistance) > POST_PAIR_MAX_DIFF_CM) {
      Serial.println("[VERIFY] Water still moving, retrying");
      firstAfterDistance = d;
      return;
    }

    float afterDistance = (d + firstAfterDistance) * 0.5f;
    float dropCm = afterDistance - beforeDrinkDistance;
    // More water used -> sonar-to-surface distance increases.
    saveWaterLevel(afterDistance);
    motionState = READY;
    firstAfterDistance = -1;

    Serial.print("[VERIFY] Before=");
    Serial.print(beforeDrinkDistance, 2);
    Serial.print("cm After=");
    Serial.print(afterDistance, 2);
    Serial.print("cm Drop=");
    Serial.print(dropCm, 2);
    Serial.println("cm");

    if (dropCm >= MIN_WATER_DROP_CM) {
      registerDrink(dropCm);
    } else {
      Serial.println("[VERIFY] No sufficient water loss, ignored");
    }
    return;
  }

  // TILTING or WAIT_UPRIGHT: do not measure while tilted.
  resetSonarBatch();
}

// --------------------------------------------------------
// SSD1306: direct I2C paging, no GFX / BusIO dependency
// --------------------------------------------------------
static const uint8_t DIGITS[10][5] = {
  {0x3E,0x51,0x49,0x45,0x3E},
  {0x00,0x42,0x7F,0x40,0x00},
  {0x42,0x61,0x51,0x49,0x46},
  {0x21,0x41,0x45,0x4B,0x31},
  {0x18,0x14,0x12,0x7F,0x10},
  {0x27,0x45,0x45,0x45,0x39},
  {0x3C,0x4A,0x49,0x49,0x30},
  {0x01,0x71,0x09,0x05,0x03},
  {0x36,0x49,0x49,0x49,0x36},
  {0x06,0x49,0x49,0x29,0x1E}
};

static const uint8_t LETTERS[26][5] = {
  {0x7E,0x11,0x11,0x11,0x7E}, // A
  {0x7F,0x49,0x49,0x49,0x36}, // B
  {0x3E,0x41,0x41,0x41,0x22}, // C
  {0x7F,0x41,0x41,0x22,0x1C}, // D
  {0x7F,0x49,0x49,0x49,0x41}, // E
  {0x7F,0x09,0x09,0x09,0x01}, // F
  {0x3E,0x41,0x49,0x49,0x7A}, // G
  {0x7F,0x08,0x08,0x08,0x7F}, // H
  {0x00,0x41,0x7F,0x41,0x00}, // I
  {0x20,0x40,0x41,0x3F,0x01}, // J
  {0x7F,0x08,0x14,0x22,0x41}, // K
  {0x7F,0x40,0x40,0x40,0x40}, // L
  {0x7F,0x02,0x0C,0x02,0x7F}, // M
  {0x7F,0x04,0x08,0x10,0x7F}, // N
  {0x3E,0x41,0x41,0x41,0x3E}, // O
  {0x7F,0x09,0x09,0x09,0x06}, // P
  {0x3E,0x41,0x51,0x21,0x5E}, // Q
  {0x7F,0x09,0x19,0x29,0x46}, // R
  {0x46,0x49,0x49,0x49,0x31}, // S
  {0x01,0x01,0x7F,0x01,0x01}, // T
  {0x3F,0x40,0x40,0x40,0x3F}, // U
  {0x1F,0x20,0x40,0x20,0x1F}, // V
  {0x7F,0x20,0x18,0x20,0x7F}, // W
  {0x63,0x14,0x08,0x14,0x63}, // X
  {0x07,0x08,0x70,0x08,0x07}, // Y
  {0x61,0x51,0x49,0x45,0x43}  // Z
};

uint8_t glyphColumn(char c, uint8_t col) {
  if (c >= 'A' && c <= 'Z') return LETTERS[c - 'A'][col];
  if (c >= '0' && c <= '9') return DIGITS[c - '0'][col];
  if (c == '%') {
    const uint8_t g[5] = {0x63,0x13,0x08,0x64,0x63};
    return g[col];
  }
  if (c == '-') return 0x08;
  if (c == '.') return (col == 2) ? 0x60 : 0x00;
  return 0;
}

void oledCommand(uint8_t cmd) {
  Wire.beginTransmission(oledAddr);
  Wire.write(0x00);
  Wire.write(cmd);
  Wire.endTransmission();
}

void oledInit() {
  oledCommand(0xAE);
  oledCommand(0xD5); oledCommand(0x80);
  oledCommand(0xA8); oledCommand(OLED_HEIGHT-1);
  oledCommand(0xD3); oledCommand(0x00);
  oledCommand(0x40);
  oledCommand(0x8D); oledCommand(0x14);
  oledCommand(0x20); oledCommand(0x02); // Page mode
  oledCommand(0xA1); oledCommand(0xC8);
  oledCommand(0xDA);
  oledCommand(OLED_HEIGHT == 32 ? 0x02 : 0x12);
  oledCommand(0x81); oledCommand(0x7F);
  oledCommand(0xD9); oledCommand(0xF1);
  oledCommand(0xDB); oledCommand(0x40);
  oledCommand(0xA4); oledCommand(0xA6);
  oledCommand(0x2E); oledCommand(0xAF);
}

uint8_t textByte(const char *text, int x, int pagePart, int scale) {
  int len = strlen(text);
  int span = len * 6 * scale;
  int start = (OLED_WIDTH-span)/2;
  int offset = x-start;
  if (offset < 0 || offset >= span) return 0;
  int index = offset/(6*scale);
  int col = (offset%(6*scale))/scale;
  if (col >= 5) return 0;
  uint8_t glyph = glyphColumn(text[index], (uint8_t)col);
  uint8_t result = 0;
  for (int i = 0; i < 8; i++) {
    int glyphRow = (pagePart*8+i)/scale;
    if (glyphRow < 7 && (glyph & (1U << glyphRow)))
      result |= (uint8_t)(1U << i);
  }
  return result;
}

void oledShow(const char *title, const char *value) {
  if (!oledOK) return;
  const int pages = OLED_HEIGHT/8;
  const int valuePage = (OLED_HEIGHT == 32) ? 1 : 3;
  for (int page = 0; page < pages; page++) {
    oledCommand(0xB0+page);
    oledCommand(0x00);
    oledCommand(0x10);
    for (int start = 0; start < OLED_WIDTH; start += 16) {
      Wire.beginTransmission(oledAddr);
      Wire.write(0x40);
      for (int i = 0; i < 16; i++) {
        int x = start+i;
        uint8_t pixel = 0;
        if (page == 0) pixel = textByte(title,x,0,1);
        if (page >= valuePage && page < valuePage+2)
          pixel = textByte(value,x,page-valuePage,2);
        Wire.write(pixel);
      }
      Wire.endTransmission();
    }
  }
}

void updateDisplay() {
  if (!oledOK) return;
  uint32_t now = millis();
  if (now - lastOledMs < DISPLAY_REFRESH_MS) return;
  lastOledMs = now;

  if (lisOK && !calibrated) {
    oledShow("GIU BIN THANG", "...");
  } else if (notifyActive && now - notifyAtMs < NOTIFY_DURATION_MS) {
    char msg[18];
    snprintf(msg, sizeof(msg), "LAN %u", (unsigned)drinkCount);
    oledShow("DA UONG NUOC", msg);
  } else {
    notifyActive = false;
    if (lisOK && motionState == TILTING) {
      oledShow("DANG NGHIENG", "...");
    } else if (lisOK && motionState == VERIFYING) {
      oledShow("DANG DO", "...");
    } else if (waterValid) {
      char msg[12];
      snprintf(msg, sizeof(msg), "%d%%", waterPercent);
      oledShow("MUC NUOC", msg);
    } else {
      oledShow("MUC NUOC", "---");
    }
  }
}

// --------------------------------------------------------
// Arduino setup / loop
// --------------------------------------------------------
void setup() {
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && millis() - start < 2000UL) delay(10);

  Serial.println("SMART WATER BOTTLE - nRF52840 SuperMini");
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);

  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  Wire.setClock(100000);
  delay(150);

  if (i2cPresent(0x3C)) oledAddr=0x3C;
  else if (i2cPresent(0x3D)) oledAddr=0x3D;
  oledOK = oledAddr != 0;
  if (oledOK) {
    oledInit();
    oledShow("HELLO", "---");
    Serial.print("OLED OK 0x"); Serial.println(oledAddr, HEX);
  } else {
    Serial.println("ERROR: OLED not found");
  }

  lisOK = initLIS3DSH();
  Serial.println(lisOK ? "LIS3DSH OK - keep bottle upright" :
                         "ERROR: LIS3DSH not found; drink counting disabled");
  delay(1000);
}

void loop() {
  updateMotion();
  updateWater();
  updateDisplay();

  uint32_t now = millis();
  if (now - lastLogMs >= 1000UL) {
    lastLogMs = now;
    Serial.print("tilt="); Serial.print(angleDeg, 1);
    Serial.print(" state="); Serial.print((int)motionState);
    Serial.print(" distance="); Serial.print(waterDistance, 2);
    Serial.print("cm water=");
    if (waterValid) Serial.print(waterPercent);
    else Serial.print("--");
    Serial.print("% drinks="); Serial.println(drinkCount);
  }
  delay(5);
}
