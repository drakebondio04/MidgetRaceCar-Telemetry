#include <Wire.h>
#include <math.h>
#include <TinyGPSPlus.h>
#include <SPI.h>
#include <SD.h>

// ---------------------------------------------------------
//   Pins & addresses
// ---------------------------------------------------------
const int PIN_I2C_SDA = 22;
const int PIN_I2C_SCL = 21;

const uint8_t MPU_ADDR = 0x68;   // MPU6500 accel/gyro
const uint8_t MAG_ADDR = 0x2C;   // HP5883 / QMC-type mag

// GPS on UART2
const int PIN_GPS_RX  = 17;      // ESP32 RX2 <- GT-U7 TX
const int PIN_GPS_TX  = 16;      // ESP32 TX2 -> GT-U7 RX

HardwareSerial SerialGPS(2);
TinyGPSPlus gps;

// SD card
const int PIN_SD_CS = 5;
File logFile;

// ---------------------------------------------------------
//   Tach pulse logging
// ---------------------------------------------------------
// Tach input from ECU "V" pin through divider
const int PIN_RPM = 34;

// ISR-visible state
volatile uint32_t tach_pulseCount = 0;       // pulses since last log
volatile uint32_t tach_lastPulseMicros = 0;  // timestamp of last pulse
volatile uint32_t tach_minDtUs = 0xFFFFFFFF; // smallest dt between pulses in current window

// Simple ISR: count *all* rising edges and track smallest dt between them
void IRAM_ATTR rpmISR() {
  uint32_t now = micros();

  if (tach_lastPulseMicros != 0) {
    uint32_t dt = now - tach_lastPulseMicros;
    if (dt < tach_minDtUs) {
      tach_minDtUs = dt;
    }
  }

  tach_lastPulseMicros = now;
  tach_pulseCount++;
}

// ---------------------------------------------------------
//   Throttle ADC (RAW percentage)
// ---------------------------------------------------------
// Throttle signal on ADC pin 33 (0–5 V divided to 0–2.5 V)
const int PIN_THROTTLE = 33;

// From your calibration:
const int THROTTLE_ADC_MIN = 1808;// 1323 47 and 1808 47x
const int THROTTLE_ADC_MAX = 2263; // 1819 47 and 2263 47x

// Raw throttle percentage (0–100%)
float throttle_pct = 0.0f;

// ---------------------------------------------------------
//   Timing
// ---------------------------------------------------------
unsigned long lastUpdateMicros = 0;
// 25 Hz debug/log print (~40 ms)
const unsigned long PRINT_INTERVAL_MS = 40;
unsigned long lastPrint = 0;

// ---------------------------------------------------------
//   IMU biases (will be auto-calibrated)
// ---------------------------------------------------------
// accel biases (g)
float ax_bias = 0.0f;
float ay_bias = 0.0f;
float az_bias = 0.0f;   // we'll force az ≈ 1 g when level

// gyro biases (deg/s)
float gx_bias = 0.0f;
float gy_bias = 0.0f;
float gz_bias = 0.0f;

// ---------------------------------------------------------
//   Filter state
// ---------------------------------------------------------
bool  rp_initialized   = false;
float roll_f           = 0.0f;
float pitch_f          = 0.0f;

// yaw
float yaw_gyro         = 0.0f;   // pure integrated gyro
float yaw_fused        = 0.0f;   // drift-corrected yaw
bool  yaw_initialized  = false;

// accel low-pass
float ax_lpf = 0.0f;
float ay_lpf = 0.0f;
float az_lpf = 1.0f;

const float accelAlpha = 0.2f;   // accel LPF
const float compAlpha  = 0.98f;  // roll/pitch complementary

// ---------------------------------------------------------
//   Magnetometer calibration (for logging only)
// ---------------------------------------------------------
float mx_bias = 0.0f, my_bias = 0.0f, mz_bias = 0.0f;
float mx_scale = 1.0f, my_scale = 1.0f, mz_scale = 1.0f;
const float magDeclinationDeg = 0.0f;   // set for your region if you care

// ---------------------------------------------------------
//   Fusion thresholds
// ---------------------------------------------------------
const float GPS_SPEED_INIT_MPH      = 5.0f;   // min speed to init yaw from GPS
const float GPS_SPEED_MIN_MPH       = 12.0f;  // min speed to use GPS corrections
const float GPS_LAT_ACC_THRESH_G    = 0.15f;  // nearly no lateral accel
const float GPS_YAWRATE_THRESH_DPS  = 25.0f;  // not turning hard

const float GPS_CORR_GAIN           = 0.15f;  // GPS correction gain

// ---------------------------------------------------------
//   Angle helpers
// ---------------------------------------------------------
float wrapAngle180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

float wrapAngle360(float a) {
  while (a >= 360.0f) a -= 360.0f;
  while (a <  0.0f)   a += 360.0f;
  return a;
}

// ---------------------------------------------------------
//   MPU6500 helpers
// ---------------------------------------------------------
void setupMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);      // PWR_MGMT_1
  Wire.write(0x00);      // wake
  Wire.endTransmission(true);
}

void readMPURaw(float &ax, float &ay, float &az,
                float &gx_deg, float &gy_deg, float &gz_deg) {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);           // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);

  int16_t ax_raw = (Wire.read() << 8) | Wire.read();
  int16_t ay_raw = (Wire.read() << 8) | Wire.read();
  int16_t az_raw = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // temp
  int16_t gx_raw = (Wire.read() << 8) | Wire.read();
  int16_t gy_raw = (Wire.read() << 8) | Wire.read();
  int16_t gz_raw = (Wire.read() << 8) | Wire.read();

  ax = ax_raw / 16384.0f;
  ay = ay_raw / 16384.0f;
  az = az_raw / 16384.0f;

  gx_deg = gx_raw / 131.0f;
  gy_deg = gy_raw / 131.0f;
  gz_deg = gz_raw / 131.0f;
}

void readMPU(float &ax, float &ay, float &az,
             float &gx_deg, float &gy_deg, float &gz_deg) {
  readMPURaw(ax, ay, az, gx_deg, gy_deg, gz_deg);
  ax     -= ax_bias;
  ay     -= ay_bias;
  az     -= az_bias;
  gx_deg -= gx_bias;
  gy_deg -= gy_bias;
  gz_deg -= gz_bias;
}

// ---------------------------------------------------------
//   Simple accel-based roll/pitch
// ---------------------------------------------------------
void getAccelRP(float ax, float ay, float az, float &roll_acc, float &pitch_acc) {
  float norm = sqrtf(ax*ax + ay*ay + az*az);
  if (norm > 1e-6f) {
    ax /= norm; ay /= norm; az /= norm;
  }
  roll_acc  = atan2f(ay, az) * 180.0f / PI;
  pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;
}

// ---------------------------------------------------------
//   Auto-calibrate IMU biases at startup (car still)
// ---------------------------------------------------------
void autoCalibrateIMU(int samples = 500) {
  Serial.println("Auto-calibration: keep car still for ~5 s...");

  float sum_ax = 0, sum_ay = 0, sum_az = 0;
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;

  float ax, ay, az, gx, gy, gz;

  // small settle period
  for (int i = 0; i < 50; i++) {
    readMPURaw(ax, ay, az, gx, gy, gz);
    delay(10);
  }

  for (int i = 0; i < samples; i++) {
    readMPURaw(ax, ay, az, gx, gy, gz);

    sum_ax += ax;
    sum_ay += ay;
    sum_az += az;

    sum_gx += gx;
    sum_gy += gy;
    sum_gz += gz;

    delay(10); // 100 Hz approx
  }

  ax_bias = sum_ax / samples;
  ay_bias = sum_ay / samples;
  az_bias = (sum_az / samples) - 1.0f;  // 1 g on Z

  gx_bias = sum_gx / samples;
  gy_bias = sum_gy / samples;
  gz_bias = sum_gz / samples;

  Serial.println("Auto-cal complete:");
  Serial.print("  ax_bias = "); Serial.println(ax_bias, 6);
  Serial.print("  ay_bias = "); Serial.println(ay_bias, 6);
  Serial.print("  az_bias = "); Serial.println(az_bias, 6);
  Serial.print("  gx_bias = "); Serial.println(gx_bias, 6);
  Serial.print("  gy_bias = "); Serial.println(gy_bias, 6);
  Serial.print("  gz_bias = "); Serial.println(gz_bias, 6);
}

// ---------------------------------------------------------
//   GPS UBX (10 Hz)
// ---------------------------------------------------------
void sendUBX(const uint8_t *msg, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) SerialGPS.write(msg[i]);
  SerialGPS.flush();
}

const uint8_t UBX_RATE_10HZ[] = {
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
  0x64, 0x00, 0x01, 0x00, 0x01, 0x00,
  0x7A, 0x12
};

// ---------------------------------------------------------
//   Magnetometer (for logging only)
// ---------------------------------------------------------
void setupMag() {
  const uint8_t MODE_REG   = 0x0A;
  const uint8_t CONFIG_REG = 0x0B;

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(MODE_REG);
  Wire.write(0xCF);     // continuous, 200 Hz, ±8G (example – adjust if needed)
  Wire.endTransmission();

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(CONFIG_REG);
  Wire.write(0x08);     // set/reset
  Wire.endTransmission();
}

bool readMag(float &mx, float &my, float &mz) {
  const uint8_t X_LSB_REG = 0x01;
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(X_LSB_REG);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) return false;

  const uint8_t toRead = 6;
  uint8_t got = Wire.requestFrom(MAG_ADDR, toRead);
  if (got != toRead) {
    while (Wire.available()) (void)Wire.read();
    return false;
  }

  uint8_t x_lsb = Wire.read();
  uint8_t x_msb = Wire.read();
  uint8_t y_lsb = Wire.read();
  uint8_t y_msb = Wire.read();
  uint8_t z_lsb = Wire.read();
  uint8_t z_msb = Wire.read();

  int16_t rawX = (int16_t)((x_msb << 8) | x_lsb);
  int16_t rawY = (int16_t)((y_msb << 8) | y_lsb);
  int16_t rawZ = (int16_t)((z_msb << 8) | z_lsb);

  mx = (rawX - mx_bias) * mx_scale;
  my = (rawY - my_bias) * my_scale;
  mz = (rawZ - mz_bias) * mz_scale;

  return true;
}

float computeMagYawDeg(float roll_deg, float pitch_deg,
                       float mx, float my, float mz) {
  float roll  = roll_deg  * PI / 180.0f;
  float pitch = pitch_deg * PI / 180.0f;

  float mx2 = mx * cosf(pitch) + mz * sinf(pitch);
  float my2 = mx * sinf(roll) * sinf(pitch)
            + my * cosf(roll)
            - mz * sinf(roll) * cosf(pitch);

  float heading = atan2f(-my2, mx2) * 180.0f / PI;
  heading += magDeclinationDeg;
  return wrapAngle360(heading);
}

// ---------------------------------------------------------
//   CSV header  (17 columns, throttle_pct added)
// ---------------------------------------------------------
void writeCSVHeader() {
  if (!logFile) return;
  logFile.println(
    "time_ms,ax,ay,az,roll,pitch,"
    "yaw_fused,yaw_gyro,yaw_mag,yaw_gps,"
    "lat,lon,spd_mph,yaw_mode,"
    "tach_pulses,tach_min_dt_us,throttle_pct"
  );
  logFile.flush();
}

// ---------------------------------------------------------
//   Setup
// ---------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nESP32 IMU + GPS + SD + Tach + Throttle (25 Hz log / 10 Hz GPS)");

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  setupMPU();
  setupMag();

  // IMU auto-cal
  autoCalibrateIMU();   // car must be still here

  // GPS
  SerialGPS.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  delay(1000);
  sendUBX(UBX_RATE_10HZ, sizeof(UBX_RATE_10HZ));
  delay(200);
  sendUBX(UBX_RATE_10HZ, sizeof(UBX_RATE_10HZ));

  // SD
  Serial.print("Mounting SD... ");
  if (!SD.begin(PIN_SD_CS)) {
    Serial.println("FAILED");
  } else {
    Serial.println("OK");
    int fileIndex = 1;
    char filename[32];
    while (true) {
      snprintf(filename, sizeof(filename), "/car_log_%d.csv", fileIndex);
      if (!SD.exists(filename)) break;
      fileIndex++;
      if (fileIndex > 9999) break;
    }
    Serial.print("Opening: "); Serial.println(filename);
    logFile = SD.open(filename, FILE_WRITE);
    if (logFile && logFile.size() == 0) writeCSVHeader();
  }

  ax_lpf = 0.0f; ay_lpf = 0.0f; az_lpf = 1.0f;

  // Tach input + interrupt
  pinMode(PIN_RPM, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_RPM), rpmISR, RISING);

  // Throttle ADC config
  analogReadResolution(12);                         // 0–4095
  analogSetPinAttenuation(PIN_THROTTLE, ADC_11db);  // up to ~3.3V

  lastUpdateMicros = micros();
  lastPrint = millis();
}

// ---------------------------------------------------------
//   Main loop
// ---------------------------------------------------------
void loop() {
  // Feed GPS parser
  while (SerialGPS.available()) gps.encode(SerialGPS.read());

  // dt
  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastUpdateMicros) / 1e6f;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;
  lastUpdateMicros = nowMicros;

  // --- IMU ---
  float ax, ay, az, gx_deg, gy_deg, gz_deg;
  readMPU(ax, ay, az, gx_deg, gy_deg, gz_deg);

  // LPF accel
  ax_lpf = accelAlpha * ax + (1.0f - accelAlpha) * ax_lpf;
  ay_lpf = accelAlpha * ay + (1.0f - accelAlpha) * ay_lpf;
  az_lpf = accelAlpha * az + (1.0f - accelAlpha) * az_lpf;

  float acc_norm   = sqrtf(ax_lpf*ax_lpf + ay_lpf*ay_lpf + az_lpf*az_lpf);
  bool  low_dynamic = fabs(acc_norm - 1.0f) < 0.15f;

  float roll_acc, pitch_acc;
  getAccelRP(ax_lpf, ay_lpf, az_lpf, roll_acc, pitch_acc);

  if (!rp_initialized) {
    roll_f = roll_acc;
    pitch_f = pitch_acc;
    rp_initialized = true;
  } else {
    float roll_gyro  = roll_f  + gx_deg * dt;
    float pitch_gyro = pitch_f + gy_deg * dt;
    if (low_dynamic) {
      roll_f  = compAlpha * roll_gyro  + (1.0f - compAlpha) * roll_acc;
      pitch_f = compAlpha * pitch_gyro + (1.0f - compAlpha) * pitch_acc;
    } else {
      roll_f  = roll_gyro;
      pitch_f = pitch_gyro;
    }
  }

  // --- MAG yaw (for logging / debugging only) ---
  float mx = 0, my = 0, mz = 0;
  bool magOK = readMag(mx, my, mz);
  float yaw_mag = 0.0f;
  if (magOK) {
    yaw_mag = computeMagYawDeg(roll_f, pitch_f, mx, my, mz);
  }

  // --- GPS ---
  double lat     = gps.location.isValid() ? gps.location.lat()  : 0.0;
  double lon     = gps.location.isValid() ? gps.location.lng()  : 0.0;
  double spd_mph = gps.speed.isValid()    ? gps.speed.mph()     : 0.0;

  bool   gpsCourseValid = gps.course.isValid();
  double gps_course_deg = gpsCourseValid ? gps.course.deg() : 0.0;
  float  yaw_gps        = wrapAngle360((float)gps_course_deg);

  // --- Yaw fusion (simple: gyro + occasional GPS correction) ---
  // integrate gyro
  yaw_gyro = wrapAngle360(yaw_gyro + gz_deg * dt);

  // initial alignment from GPS once moving straight-ish
  if (!yaw_initialized && gpsCourseValid && (spd_mph > GPS_SPEED_INIT_MPH)) {
    yaw_gyro    = yaw_gps;
    yaw_fused   = yaw_gps;
    yaw_initialized = true;
  }

  // default: gyro-only
  yaw_fused = yaw_gyro;
  uint8_t yaw_mode = 0;  // 0=gyro, 1=GPS-corrected

  // conditions to trust GPS (no big cornering)
  float lat_g = ay_lpf; // approx lateral acceleration in body frame
  bool low_lat      = fabs(lat_g)  < GPS_LAT_ACC_THRESH_G;
  bool low_yaw_rate = fabs(gz_deg) < GPS_YAWRATE_THRESH_DPS;

  bool useGPS = yaw_initialized &&
                gpsCourseValid &&
                (spd_mph > GPS_SPEED_MIN_MPH) &&
                low_lat && low_yaw_rate;

  if (useGPS) {
    float diff = wrapAngle180(yaw_gps - yaw_gyro);
    yaw_fused = wrapAngle360(yaw_gyro + GPS_CORR_GAIN * diff);
    yaw_mode  = 1;
  }

  // --- Raw Throttle % (no smoothing) ---
  int adc_raw_throttle = analogRead(PIN_THROTTLE);

  // Clamp into calibrated range
  int adc_clamped = constrain(adc_raw_throttle,
                              THROTTLE_ADC_MIN,
                              THROTTLE_ADC_MAX);

  throttle_pct = (float)(adc_clamped - THROTTLE_ADC_MIN) * 100.0f /
                 (float)(THROTTLE_ADC_MAX - THROTTLE_ADC_MIN);

  // --- Debug print & logging @ 25 Hz ---
  unsigned long nowMs = millis();
  if (nowMs - lastPrint >= PRINT_INTERVAL_MS) {
    float dt_s = (nowMs - lastPrint) / 1000.0f;
    lastPrint = nowMs;

    // Snapshot tach state atomically
    uint32_t pulses;
    uint32_t minDtUs;
    noInterrupts();
    pulses  = tach_pulseCount;
    minDtUs = tach_minDtUs;
    tach_pulseCount = 0;
    tach_minDtUs = 0xFFFFFFFF;
    interrupts();

    // --- Serial debug ---
    Serial.print("aLPF: ax="); Serial.print(ax_lpf,3);
    Serial.print(" ay=");      Serial.print(ay_lpf,3);
    Serial.print(" az=");      Serial.print(az_lpf,3);
    Serial.print(" | |a|=");   Serial.print(acc_norm,3);

    Serial.print(" | roll=");      Serial.print(roll_f,1);
    Serial.print(" pitch=");       Serial.print(pitch_f,1);
    Serial.print(" yaw_gyro=");    Serial.print(yaw_gyro,1);
    Serial.print(" yaw_fused=");   Serial.print(yaw_fused,1);
    Serial.print(" yaw_mag=");     Serial.print(yaw_mag,1);
    Serial.print(" yaw_gps=");     Serial.print(yaw_gps,1);
    Serial.print(" mode=");        Serial.print(yaw_mode);

    Serial.print(" | lat=");       Serial.print(lat,6);
    Serial.print(" lon=");         Serial.print(lon,6);
    Serial.print(" spd=");         Serial.print(spd_mph,1);

    Serial.print(" | tach_pulses=");   Serial.print(pulses);
    Serial.print(" min_dt_us=");
    if (minDtUs == 0xFFFFFFFF) Serial.print("NA");
    else                       Serial.print(minDtUs);

    Serial.print(" | throttle_pct=");
    Serial.print(throttle_pct, 1);

    Serial.print(" dt_s="); Serial.print(dt_s,3);
    Serial.println();

    // --- Logging to SD (17 columns) ---
    if (logFile) {
      logFile.print(nowMs);        logFile.print(',');
      logFile.print(ax_lpf,3);     logFile.print(',');
      logFile.print(ay_lpf,3);     logFile.print(',');
      logFile.print(az_lpf,3);     logFile.print(',');
      logFile.print(roll_f,1);     logFile.print(',');
      logFile.print(pitch_f,1);    logFile.print(',');
      logFile.print(yaw_fused,1);  logFile.print(',');
      logFile.print(yaw_gyro,1);   logFile.print(',');
      logFile.print(yaw_mag,1);    logFile.print(',');
      logFile.print(yaw_gps,1);    logFile.print(',');
      logFile.print(lat,6);        logFile.print(',');
      logFile.print(lon,6);        logFile.print(',');
      logFile.print(spd_mph,1);    logFile.print(',');
      logFile.print((int)yaw_mode);logFile.print(',');

      // tach pulses & smallest dt
      logFile.print(pulses);       logFile.print(',');
      if (minDtUs == 0xFFFFFFFF) {
        logFile.print(0);          // 0 = "no pulses" sentinel
      } else {
        logFile.print(minDtUs);
      }
      logFile.print(',');

      // Raw throttle %
      logFile.println(throttle_pct, 1);

      logFile.flush();
    }
  }
}
