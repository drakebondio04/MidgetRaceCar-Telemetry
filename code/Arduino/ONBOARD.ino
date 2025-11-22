#include <Wire.h>
#include <math.h>
#include <TinyGPSPlus.h>
#include <SPI.h>
#include <SD.h>

// ---------------------------------------------------------
//   Pins & addresses
// ---------------------------------------------------------
const int PIN_I2C_SDA = 22;      // I2C SDA
const int PIN_I2C_SCL = 21;      // I2C SCL

const uint8_t MPU_ADDR = 0x68;   // MPU6500 accel/gyro

// GPS on UART2 (Serial2) - MATCHING YOUR WORKING ECHO CODE
const int PIN_GPS_RX  = 17;      // ESP32 RX2 <- GT-U7 TX
const int PIN_GPS_TX  = 16;      // ESP32 TX2 -> GT-U7 RX (optional)

HardwareSerial SerialGPS(2);
TinyGPSPlus gps;

// SD card (SPI) - Option A: CS=5, CLK=18, MISO=19, MOSI=23
const int PIN_SD_CS = 5;
File logFile;

// ---------------------------------------------------------
//   Timing
// ---------------------------------------------------------
unsigned long lastUpdateMicros = 0;
const unsigned long PRINT_INTERVAL_MS = 100; // 10 Hz output
unsigned long lastPrint = 0;

// ---------------------------------------------------------
//   Calibration samples
// ---------------------------------------------------------
const int CALIB_SAMPLES = 2000;

// ---------------------------------------------------------
//   Calibration values (your measured biases)
// ---------------------------------------------------------
// accel biases (g)
float ax_bias = 0.062711;
float ay_bias = 0.060081;
float az_bias = -0.073106;   // we’ll force az ≈ 1 g when level

// gyro biases (deg/s)
float gx_bias = -1.835813;
float gy_bias = -0.269863;
float gz_bias = 0.228622;

// ---------------------------------------------------------
//   Complementary filter state
// ---------------------------------------------------------
bool rp_initialized = false;   // did we initialize roll/pitch from accel?
float roll_f  = 0.0f;          // filtered roll (deg)
float pitch_f = 0.0f;          // filtered pitch (deg)
float yaw_f   = 0.0f;          // filtered yaw (deg, gyro-only, will drift)

// higher = more trust in gyro, lower = more trust in accel
const float compAlpha = 0.98f;

// Low-pass filter on accel to reduce vibration noise
float ax_lpf = 0.0f;
float ay_lpf = 0.0f;
float az_lpf = 1.0f;   // start near gravity
const float accelAlpha = 0.2f;   // 0..1, smaller = smoother

// ---------------------------------------------------------
//   MPU6500 low-level functions
// ---------------------------------------------------------
void setupMPU() {
  // Wake up the device (clear sleep bit)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);      // PWR_MGMT_1
  Wire.write(0x00);      // set to 0 -> wake up
  Wire.endTransmission(true);

  // Optional: set accel/gyro ranges explicitly if you want.
  // For default config: ±2g accel, ±250 dps gyro.
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
  Wire.read(); Wire.read(); // temp, ignore
  int16_t gx_raw = (Wire.read() << 8) | Wire.read();
  int16_t gy_raw = (Wire.read() << 8) | Wire.read();
  int16_t gz_raw = (Wire.read() << 8) | Wire.read();

  // Convert to physical units
  ax = ax_raw / 16384.0f;    // g (±2g)
  ay = ay_raw / 16384.0f;
  az = az_raw / 16384.0f;

  gx_deg = gx_raw / 131.0f;  // deg/s (±250 dps)
  gy_deg = gy_raw / 131.0f;
  gz_deg = gz_raw / 131.0f;
}

void readMPU(float &ax, float &ay, float &az,
             float &gx_deg, float &gy_deg, float &gz_deg) {
  readMPURaw(ax, ay, az, gx_deg, gy_deg, gz_deg);

  // Apply bias calibration
  ax -= ax_bias;
  ay -= ay_bias;
  az -= az_bias;

  gx_deg -= gx_bias;
  gy_deg -= gy_bias;
  gz_deg -= gz_bias;
}

// ---------------------------------------------------------
//   Accel-only roll / pitch (tilt from gravity)
// ---------------------------------------------------------
void getAccelRP(float ax, float ay, float az, float &roll_acc, float &pitch_acc) {
  // Optional: normalize accel vector
  float norm = sqrtf(ax*ax + ay*ay + az*az);
  if (norm > 1e-6f) {
    ax /= norm;
    ay /= norm;
    az /= norm;
  }

  // roll: rotation about X-axis (car rolls left/right)
  roll_acc = atan2f(ay, az) * 180.0f / PI;

  // pitch: rotation about Y-axis (nose up/down)
  pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;
}

// ---------------------------------------------------------
//   Optional calibration routine
// ---------------------------------------------------------
void calibrateIMU() {
  Serial.println();
  Serial.println("=== IMU Calibration ===");
  Serial.println("Place the car/board LEVEL and COMPLETELY STILL.");
  Serial.println("Calibration will start in 3 seconds...");
  delay(3000);

  float ax, ay, az, gx, gy, gz;
  for (int i = 0; i < 200; i++) {
    readMPURaw(ax, ay, az, gx, gy, gz);
    delay(2);
  }

  Serial.println("Collecting calibration samples...");
  double sum_ax = 0, sum_ay = 0, sum_az = 0;
  double sum_gx = 0, sum_gy = 0, sum_gz = 0;

  for (int i = 0; i < CALIB_SAMPLES; i++) {
    readMPURaw(ax, ay, az, gx, gy, gz);

    sum_ax += ax;
    sum_ay += ay;
    sum_az += az;

    sum_gx += gx;
    sum_gy += gy;
    sum_gz += gz;

    delay(2);
  }

  float mean_ax = sum_ax / CALIB_SAMPLES;
  float mean_ay = sum_ay / CALIB_SAMPLES;
  float mean_az = sum_az / CALIB_SAMPLES;

  float mean_gx = sum_gx / CALIB_SAMPLES;
  float mean_gy = sum_gy / CALIB_SAMPLES;
  float mean_gz = sum_gz / CALIB_SAMPLES;

  ax_bias = mean_ax;
  ay_bias = mean_ay;
  az_bias = mean_az - 1.0f;

  gx_bias = mean_gx;
  gy_bias = mean_gy;
  gz_bias = mean_gz;

  Serial.println("Calibration complete. Biases:");
  Serial.print("  ax_bias = "); Serial.println(ax_bias, 6);
  Serial.print("  ay_bias = "); Serial.println(ay_bias, 6);
  Serial.print("  az_bias = "); Serial.println(az_bias, 6);

  Serial.print("  gx_bias = "); Serial.println(gx_bias, 6);
  Serial.print("  gy_bias = "); Serial.println(gy_bias, 6);
  Serial.print("  gz_bias = "); Serial.println(gz_bias, 6);
  Serial.println("========================");
}

// ---------------------------------------------------------
//   UBX helper: set u-blox update rate to 10 Hz
//   UBX-CFG-RATE: measRate=100 ms, navRate=1, timeRef=1 (GPS time)
//   Message: B5 62 06 08 06 00 64 00 01 00 01 00 7A 12
// ---------------------------------------------------------
void sendUBX(const uint8_t *msg, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    SerialGPS.write(msg[i]);
  }
  SerialGPS.flush();
}

const uint8_t UBX_RATE_10HZ[] = {
  0xB5, 0x62,             // sync chars
  0x06, 0x08,             // class = CFG, id = RATE
  0x06, 0x00,             // length = 6
  0x64, 0x00,             // measRate = 100ms (0x0064)
  0x01, 0x00,             // navRate = 1
  0x01, 0x00,             // timeRef = 1 (GPS time)
  0x7A, 0x12              // CK_A, CK_B
};

// ---------------------------------------------------------
//   SD helpers: auto-numbered CSV files
// ---------------------------------------------------------
String generateNewFilename() {
  int fileIndex = 1;

  // Look for next available filename up to 999
  while (fileIndex < 1000) {
    char filename[20];
    sprintf(filename, "/log_%03d.csv", fileIndex);

    if (!SD.exists(filename)) {
      Serial.print("Creating new log file: ");
      Serial.println(filename);
      return String(filename);
    }

    fileIndex++;
  }

  // Fallback — shouldn’t really happen
  Serial.println("WARNING: Reached log_999.csv, using it.");
  return String("/log_999.csv");
}

void writeCSVHeader() {
  if (!logFile) return;
  logFile.println("time_ms,ax,ay,az,roll,pitch,yaw,lat,lon,spd_mph");
  logFile.flush();
}

// ---------------------------------------------------------
//   Setup
// ---------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("ESP32 Dirt Track IMU + GPS (10 Hz) + SD: roll, pitch, yaw, lat, lon, mph");

  // I2C for IMU
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000); // 400 kHz I2C
  setupMPU();

  // GPS on Serial2, using working RX/TX pins
  SerialGPS.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  Serial.println("GPS serial started at 9600 baud on RX=17 TX=16...");

  // Give GPS a bit of time to boot before sending UBX config
  delay(1000);
  Serial.println("Sending UBX command to set GPS rate to 10 Hz...");
  sendUBX(UBX_RATE_10HZ, sizeof(UBX_RATE_10HZ));
  delay(200);
  sendUBX(UBX_RATE_10HZ, sizeof(UBX_RATE_10HZ));
  Serial.println("UBX CFG-RATE 10 Hz command sent.");

  // SD card
  Serial.print("Mounting SD card... ");
  if (!SD.begin(PIN_SD_CS)) {
    Serial.println("FAILED");
  } else {
    Serial.println("OK");

    // Create new numbered log file: /log_001.csv, /log_002.csv, ...
    String fname = generateNewFilename();
    logFile = SD.open(fname.c_str(), FILE_WRITE);
    if (!logFile) {
      Serial.println("Failed to open new log file!");
    } else {
      Serial.print("Logging to: ");
      Serial.println(fname);
      writeCSVHeader();
    }
  }

  // Optional: run IMU calibration if you want to re-measure biases
  // calibrateIMU();

  // Init LPF accel around "gravity up"
  ax_lpf = 0.0f;
  ay_lpf = 0.0f;
  az_lpf = 1.0f;

  lastUpdateMicros = micros();
  lastPrint = millis();
}

// ---------------------------------------------------------
//   Main loop
// ---------------------------------------------------------
void loop() {
  // --- Feed GPS parser continuously ---
  while (SerialGPS.available()) {
    gps.encode(SerialGPS.read());
  }

  // --- Time step (dt in seconds) ---
  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastUpdateMicros) / 1e6f;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.01f; // clamp
  lastUpdateMicros = nowMicros;

  // --- Read IMU with biases applied ---
  float ax, ay, az;
  float gx_deg, gy_deg, gz_deg;
  readMPU(ax, ay, az, gx_deg, gy_deg, gz_deg);

  // --- Low-pass filter accel to reduce harsh vibration ---
  ax_lpf = accelAlpha * ax + (1.0f - accelAlpha) * ax_lpf;
  ay_lpf = accelAlpha * ay + (1.0f - accelAlpha) * ay_lpf;
  az_lpf = accelAlpha * az + (1.0f - accelAlpha) * az_lpf;

  // --- Accel-only roll/pitch from filtered accel ---
  float roll_acc, pitch_acc;
  getAccelRP(ax_lpf, ay_lpf, az_lpf, roll_acc, pitch_acc);

  // --- Complementary filter for smooth roll/pitch/yaw ---
  if (!rp_initialized) {
    roll_f  = roll_acc;
    pitch_f = pitch_acc;
    yaw_f   = 0.0f;      // zero yaw at startup
    rp_initialized = true;
  } else {
    float roll_gyro  = roll_f  + gx_deg * dt;
    float pitch_gyro = pitch_f + gy_deg * dt;
    float yaw_gyro   = yaw_f   + gz_deg * dt;

    roll_f  = compAlpha * roll_gyro  + (1.0f - compAlpha) * roll_acc;
    pitch_f = compAlpha * pitch_gyro + (1.0f - compAlpha) * pitch_acc;

    // Yaw: gyro-only (no mag) — will slowly drift
    yaw_f = yaw_gyro;
  }

  // --- Gather GPS data (latest parsed values) ---
  double lat     = gps.location.isValid() ? gps.location.lat()  : 0.0;
  double lon     = gps.location.isValid() ? gps.location.lng()  : 0.0;
  double spd_mph = gps.speed.isValid()    ? gps.speed.mph()     : 0.0;  // mph

  // --- Periodic print & log (10 Hz) ---
  unsigned long nowMs = millis();
  if (nowMs - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = nowMs;

    // Serial debug
    Serial.print("aLPF: ax="); Serial.print(ax_lpf, 3);
    Serial.print(" ay=");      Serial.print(ay_lpf, 3);
    Serial.print(" az=");      Serial.print(az_lpf, 3);

    Serial.print(" | roll_f=");  Serial.print(roll_f, 1);
    Serial.print(" pitch_f=");   Serial.print(pitch_f, 1);
    Serial.print(" yaw_f=");     Serial.print(yaw_f, 1);

    Serial.print(" | lat=");     Serial.print(lat, 6);
    Serial.print(" lon=");       Serial.print(lon, 6);
    Serial.print(" spd_mph=");   Serial.print(spd_mph, 1);
    Serial.println();

    // SD CSV: time_ms,ax,ay,az,roll,pitch,yaw,lat,lon,spd_mph
    if (logFile) {
      logFile.print(nowMs);          logFile.print(',');
      logFile.print(ax_lpf, 3);      logFile.print(',');
      logFile.print(ay_lpf, 3);      logFile.print(',');
      logFile.print(az_lpf, 3);      logFile.print(',');
      logFile.print(roll_f, 1);      logFile.print(',');
      logFile.print(pitch_f, 1);     logFile.print(',');
      logFile.print(yaw_f, 1);       logFile.print(',');
      logFile.print(lat, 6);         logFile.print(',');
      logFile.print(lon, 6);         logFile.print(',');
      logFile.println(spd_mph, 1);

      logFile.flush();  // simple but safe; 10 Hz is fine
    }
  }
}
