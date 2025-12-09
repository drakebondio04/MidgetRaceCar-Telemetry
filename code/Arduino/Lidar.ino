#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <LIDARLite.h>
#include <SPI.h>
#include <SD.h>

LIDARLite lidar;
int cal_cnt = 0;

// WiFi Access Point credentials
const char* ssid = "LIDAR-ESP";
const char* password = "12345678";

WebServer server(80);

// ----- SD card config -----
const int PIN_SD_CS = 5;   // <-- CHANGE to your SD CS pin
bool sdOk = false;
File logFile;
String currentFilename = "";
String currentPrefix = "";   // "midget47" or "midget47x"
int lapCounter = 0;

// Per-car file index counters
int midget47Index = 0;
int midget47xIndex = 0;

// ----- Lap timing & calibration state -----
int referenceDist = 0;
bool refSet = false;

const int detectionThreshold = 30;           // cm difference from reference to count as "object passing"
const unsigned long MIN_LAP_TIME_MS = 1000;  // ignore laps shorter than this
const unsigned long SAMPLE_INTERVAL_US = 2000; // ~500 Hz (2 ms between samples)

bool objectPresent = false;
bool lastObjectPresent = false;

bool lapRunning = false;
bool hasStarted = false;      // has first crossing occurred after Go?
unsigned long lapStartMs = 0;
unsigned long lastLapMs = 0;

// "Go" arming: only react to distance changes when armed
bool armed = false;

// latest measured distance for status endpoint
int currentDistance = 0;

// ----- In-memory lap history for display -----
const int MAX_LAPS_IN_MEMORY = 20;
int lapNumbers[MAX_LAPS_IN_MEMORY];
unsigned long lapTimesMsArr[MAX_LAPS_IN_MEMORY];
int lapHistoryCount = 0;

// Forward declarations
int readDistance();
void updateLapLogic(int dist, unsigned long now);
void logLap(unsigned long lapMs);
String makeFilename(const String& prefix, int index);
void openNewCsv(const String& prefix, int index);
bool clearSdCard();
void addLapToHistory(int lapNum, unsigned long lapMs);
void startNewCsvMidget47();
void startNewCsvMidget47x();

// HTML page served to users
String webpage = R"=====(
<!DOCTYPE html>
<html>
<head>
  <title>LIDAR Lap Timer</title>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; margin-top: 40px; background: #f5f5f5; }
    h1 { font-size: 36px; margin-bottom: 10px; }
    h2 { font-size: 20px; margin-top: 30px; }
    #dist { font-size: 48px; color: #0077FF; margin: 10px 0; }
    #ref { font-size: 20px; margin: 10px 0; }
    #lap { font-size: 32px; margin: 10px 0; }
    #lastLap { font-size: 20px; margin: 10px 0; color: #444; }
    #sdStatus { font-size: 16px; margin-top: 15px; color: #333; }
    #armedStatus { font-size: 16px; margin-top: 5px; color: #333; }
    #lapList { margin-top: 20px; text-align: left; display: inline-block; }
    button {
      font-size: 18px;
      padding: 10px 20px;
      margin: 10px;
      border-radius: 6px;
      border: none;
      cursor: pointer;
      background: #0077FF;
      color: white;
    }
    button:hover { background: #005FCC; }
  </style>
</head>
<body>
  <h1>LIDAR Lap Timer</h1>
  <div id="dist">Distance: -- cm</div>
  <div id="ref">Reference: Not set</div>

  <button onclick="setReference()">Set Reference Distance</button>
  <button onclick="go()">Go (Arm)</button>
  <button onclick="resetLap()">Reset Lap</button>
  <br />
  <button onclick="newCsv47()">New CSV (Midget47)</button>
  <button onclick="newCsv47x()">New CSV (Midget47X)</button>
  <button onclick="clearSd()">Clear SD Card</button>

  <div id="armedStatus">Armed: false</div>

  <h2>Lap Timing</h2>
  <div id="lap">Current Lap: -- s</div>
  <div id="lastLap">Last Lap: -- s</div>

  <div id="sdStatus">SD: unknown</div>

  <div id="lapList">
    <h2>Lap History</h2>
    <div id="lapHistoryContent">No laps yet.</div>
  </div>

  <script>
    async function fetchStatus() {
      try {
        let res = await fetch("/status");
        if (!res.ok) return;
        let data = await res.json();

        document.getElementById("dist").innerHTML = "Distance: " + data.dist + " cm";

        if (data.refSet) {
          document.getElementById("ref").innerHTML = "Reference: " + data.ref + " cm";
        } else {
          document.getElementById("ref").innerHTML = "Reference: Not set";
        }

        if (data.lapRunning) {
          document.getElementById("lap").innerHTML =
            "Current Lap: " + (data.lapTimeMs / 1000).toFixed(2) + " s (running)";
        } else {
          if (data.lastLapMs > 0) {
            document.getElementById("lap").innerHTML = "Current Lap: -- s (stopped)";
            document.getElementById("lastLap").innerHTML =
              "Last Lap: " + (data.lastLapMs / 1000).toFixed(2) + " s";
          } else {
            document.getElementById("lap").innerHTML = "Current Lap: -- s";
            document.getElementById("lastLap").innerHTML = "Last Lap: -- s";
          }
        }

        let sdText = "SD: " + (data.sdOk ? "OK" : "NOT AVAILABLE");
        if (data.sdOk && data.currentFile && data.currentFile.length > 0) {
          sdText += " | File: " + data.currentFile;
        }
        document.getElementById("sdStatus").innerHTML = sdText;

        document.getElementById("armedStatus").innerHTML =
          "Armed: " + (data.armed ? "true" : "false");

        // Lap history list
        let lapHtml = "";
        if (data.laps && data.laps.length > 0) {
          lapHtml += "<ul>";
          for (let i = 0; i < data.laps.length; i++) {
            let lap = data.laps[i];
            lapHtml += "<li>Lap " + lap.n + ": " +
              (lap.t / 1000).toFixed(3) + " s</li>";
          }
          lapHtml += "</ul>";
        } else {
          lapHtml = "No laps yet.";
        }
        document.getElementById("lapHistoryContent").innerHTML = lapHtml;

      } catch (e) {
        console.log(e);
      }
    }

    async function setReference() {
      try {
        let res = await fetch("/setRef");
        if (res.ok) {
          alert("Reference distance set!");
        } else {
          alert("Error setting reference");
        }
      } catch (e) {
        alert("Error setting reference");
      }
    }

    async function resetLap() {
      try {
        await fetch("/resetLap");
      } catch (e) {
        console.log(e);
      }
    }

    async function newCsv47() {
      try {
        let res = await fetch("/newCsv47");
        if (res.ok) {
          let text = await res.text();
          alert("New Midget47 CSV: " + text);
        } else {
          alert("Error creating Midget47 CSV (check SD card)");
        }
      } catch (e) {
        alert("Error creating Midget47 CSV");
      }
    }

    async function newCsv47x() {
      try {
        let res = await fetch("/newCsv47x");
        if (res.ok) {
          let text = await res.text();
          alert("New Midget47X CSV: " + text);
        } else {
          alert("Error creating Midget47X CSV (check SD card)");
        }
      } catch (e) {
        alert("Error creating Midget47X CSV");
      }
    }

    async function clearSd() {
      if (!confirm("Are you sure you want to delete ALL files on the SD card?")) return;
      try {
        let res = await fetch("/clearSd");
        let text = await res.text();
        alert(text);
      } catch (e) {
        alert("Error clearing SD card");
      }
    }

    async function go() {
      try {
        let res = await fetch("/go");
        if (!res.ok) {
          let txt = await res.text();
          alert("Error arming system: " + txt);
        }
      } catch (e) {
        alert("Error arming system");
      }
    }

    // UI poll rate (doesn't affect LIDAR sample rate)
    setInterval(fetchStatus, 100);
    fetchStatus();
  </script>
</body>
</html>
)=====";

// ----- Utility: read LIDAR distance with bias every 100th measurement -----
int readDistance() {
  int dist;
  if (cal_cnt == 0) {
    dist = lidar.distance();        // with bias correction
  } else {
    dist = lidar.distance(false);   // without bias
  }
  cal_cnt = (cal_cnt + 1) % 100;
  return dist;
}

// ----- Helper: make filename with 2-digit index -----
String makeFilename(const String& prefix, int index) {
  String idxStr = String(index);
  if (index < 10) idxStr = "0" + idxStr;
  return "/" + prefix + "_" + idxStr + ".csv";
}

// ----- SD: open a new CSV file for given prefix/index and write header -----
void openNewCsv(const String& prefix, int index) {
  if (!sdOk) return;

  if (logFile) {
    logFile.close();
  }

  currentPrefix = prefix;
  currentFilename = makeFilename(prefix, index);
  logFile = SD.open(currentFilename, FILE_WRITE);

  if (logFile) {
    Serial.print("Opened new CSV: ");
    Serial.println(currentFilename);
    logFile.println("lap_number,lap_time_ms,lap_time_s,reference_cm,detection_threshold_cm");
    logFile.flush();
    lapCounter = 0;
    lapHistoryCount = 0;
  } else {
    Serial.print("Failed to open new CSV file: ");
    Serial.println(currentFilename);
    currentFilename = "";
    currentPrefix = "";
  }
}

// ----- Per-car wrappers -----
void startNewCsvMidget47() {
  if (!sdOk) return;
  midget47Index++;
  openNewCsv("midget47", midget47Index);
}

void startNewCsvMidget47x() {
  if (!sdOk) return;
  midget47xIndex++;
  openNewCsv("midget47x", midget47xIndex);
}

// ----- SD: clear all files in root -----
bool clearSdCard() {
  if (!sdOk) return false;

  if (logFile) {
    logFile.close();
  }

  File root = SD.open("/");
  if (!root) {
    Serial.println("Failed to open SD root");
    return false;
  }

  File file = root.openNextFile();
  while (file) {
    String name = file.name();
    if (!file.isDirectory()) {
      Serial.print("Removing: ");
      Serial.println(name);
      SD.remove(name.c_str());
    }
    file = root.openNextFile();
  }
  root.close();

  currentFilename = "";
  currentPrefix = "";
  lapCounter = 0;
  lapHistoryCount = 0;
  midget47Index = 0;
  midget47xIndex = 0;
  return true;
}

// ----- Maintain in-memory lap history -----
void addLapToHistory(int lapNum, unsigned long lapMs) {
  if (lapHistoryCount < MAX_LAPS_IN_MEMORY) {
    lapNumbers[lapHistoryCount] = lapNum;
    lapTimesMsArr[lapHistoryCount] = lapMs;
    lapHistoryCount++;
  } else {
    // shift everything left, drop oldest
    for (int i = 1; i < MAX_LAPS_IN_MEMORY; i++) {
      lapNumbers[i - 1] = lapNumbers[i];
      lapTimesMsArr[i - 1] = lapTimesMsArr[i];
    }
    lapNumbers[MAX_LAPS_IN_MEMORY - 1] = lapNum;
    lapTimesMsArr[MAX_LAPS_IN_MEMORY - 1] = lapMs;
  }
}

// ----- Log a completed lap to the current CSV -----
void logLap(unsigned long lapMs) {
  if (!sdOk || !logFile) return;

  lapCounter++;
  float lapSec = lapMs / 1000.0f;

  logFile.print(lapCounter);
  logFile.print(",");
  logFile.print(lapMs);
  logFile.print(",");
  logFile.print(lapSec, 3);
  logFile.print(",");
  logFile.print(referenceDist);
  logFile.print(",");
  logFile.println(detectionThreshold);
  logFile.flush();

  addLapToHistory(lapCounter, lapMs);

  Serial.print("Logged lap ");
  Serial.print(lapCounter);
  Serial.print(" : ");
  Serial.print(lapSec, 3);
  Serial.println(" s");
}

// ----- Update lap logic based on distance -----
// Runs at high rate (~500 Hz) from loop()
void updateLapLogic(int dist, unsigned long now) {
  if (!refSet || !armed) return;

  lastObjectPresent = objectPresent;
  // "Object present" if distance differs from reference by more than threshold
  objectPresent = (abs(dist - referenceDist) > detectionThreshold);

  // Rising edge: clear -> object present
  if (!lastObjectPresent && objectPresent) {
    if (!hasStarted) {
      // First crossing after Go: start timing
      hasStarted = true;
      lapRunning = true;
      lapStartMs = now;
      Serial.println("Lap timing started (first crossing)");
    } else {
      // Continuous mode: this crossing ends previous lap and starts next
      unsigned long lapMs = now - lapStartMs;

      // Ignore spurious very short laps
      if (lapMs < MIN_LAP_TIME_MS) {
        Serial.print("Ignored spurious lap (");
        Serial.print(lapMs);
        Serial.println(" ms < MIN_LAP_TIME_MS)");
        // Do NOT change lapStartMs; keep timing the same lap
        return;
      }

      // Valid lap: log it
      lastLapMs = lapMs;
      logLap(lapMs);

      // Immediately start next lap from this crossing
      lapStartMs = now;
      lapRunning = true;

      Serial.print("New lap started, waiting for next crossing. Next lap will be #");
      Serial.println(lapCounter + 1);
    }
  }
}

// Handle root page
void handleRoot() {
  server.send(200, "text/html", webpage);
}

// Handle status endpoint (distance + lap info + SD info + history)
void handleStatus() {
  unsigned long now = millis();

  unsigned long currentLapMs = 0;
  if (lapRunning && refSet && armed && hasStarted) {
    currentLapMs = now - lapStartMs;
  }

  String json = "{";
  json += "\"dist\":" + String(currentDistance) + ",";
  json += "\"ref\":" + String(referenceDist) + ",";
  json += "\"refSet\":" + String(refSet ? "true" : "false") + ",";
  json += "\"lapRunning\":" + String(lapRunning ? "true" : "false") + ",";
  json += "\"lapTimeMs\":" + String(currentLapMs) + ",";
  json += "\"lastLapMs\":" + String(lastLapMs) + ",";
  json += "\"sdOk\":" + String(sdOk ? "true" : "false") + ",";
  json += "\"currentFile\":\"" + currentFilename + "\",";
  json += "\"armed\":" + String(armed ? "true" : "false") + ",";
  json += "\"laps\":[";
  for (int i = 0; i < lapHistoryCount; i++) {
    if (i > 0) json += ",";
    json += "{\"n\":" + String(lapNumbers[i]) +
            ",\"t\":" + String(lapTimesMsArr[i]) + "}";
  }
  json += "]";
  json += "}";

  server.send(200, "application/json", json);
}

// Handle setting reference distance (calibration)
void handleSetRef() {
  int dist = readDistance();
  referenceDist = dist;
  refSet = true;

  // Reset lap & arm state when we recalibrate
  lapRunning = false;
  lastLapMs = 0;
  lapStartMs = 0;
  objectPresent = false;
  lastObjectPresent = false;
  armed = false;
  hasStarted = false;

  lapHistoryCount = 0;
  lapCounter = 0;

  server.send(200, "text/plain", "OK");
}

// Reset lap without touching reference
void handleResetLap() {
  lapRunning = false;
  lastLapMs = 0;
  lapStartMs = 0;
  objectPresent = false;
  lastObjectPresent = false;
  armed = false;   // also disarm so you must press Go again
  hasStarted = false;

  server.send(200, "text/plain", "OK");
}

// Start a new CSV file for Midget47
void handleNewCsv47() {
  if (!sdOk) {
    server.send(500, "text/plain", "SD not initialized");
    return;
  }

  startNewCsvMidget47();
  if (logFile) {
    server.send(200, "text/plain", currentFilename);
  } else {
    server.send(500, "text/plain", "Failed to create CSV for Midget47");
  }
}

// Start a new CSV file for Midget47X
void handleNewCsv47x() {
  if (!sdOk) {
    server.send(500, "text/plain", "SD not initialized");
    return;
  }

  startNewCsvMidget47x();
  if (logFile) {
    server.send(200, "text/plain", currentFilename);
  } else {
    server.send(500, "text/plain", "Failed to create CSV for Midget47X");
  }
}

// Clear SD card
void handleClearSd() {
  if (!sdOk) {
    server.send(500, "text/plain", "SD not initialized");
    return;
  }

  bool ok = clearSdCard();
  if (ok) {
    server.send(200, "text/plain", "SD card cleared");
  } else {
    server.send(500, "text/plain", "Failed to clear SD card");
  }
}

// Go / arm endpoint
void handleGo() {
  if (!refSet) {
    server.send(400, "text/plain", "Reference not set");
    return;
  }

  // Arm system; wait for first distance change to start timing
  armed = true;
  lapRunning = false;
  lastLapMs = 0;
  lapStartMs = 0;
  objectPresent = false;
  lastObjectPresent = false;
  hasStarted = false;

  server.send(200, "text/plain", "Armed");
}

void setup() {
  Serial.begin(115200);

  // LIDAR init
  Wire.begin(21, 22);
  Wire.setClock(400000);
  lidar.begin(0, true);
  lidar.configure(0);

  // SD init
  Serial.println("Initializing SD card...");
  if (SD.begin(PIN_SD_CS)) {
    sdOk = true;
    Serial.println("SD card initialized");
    // Note: no default CSV opened here; use the buttons to create midget47 / midget47x files.
  } else {
    sdOk = false;
    Serial.println("SD card initialization FAILED");
  }

  // WiFi Access Point mode
  Serial.println("Starting AP...");
  WiFi.softAP(ssid, password);
  Serial.println("AP Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Web server routes
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/setRef", handleSetRef);
  server.on("/resetLap", handleResetLap);
  server.on("/newCsv47", handleNewCsv47);
  server.on("/newCsv47x", handleNewCsv47x);
  server.on("/clearSd", handleClearSd);
  server.on("/go", handleGo);

  server.begin();
  Serial.println("Web server started");
}

void loop() {
  // High-rate LIDAR sampling & lap logic (~500 Hz)
  static unsigned long lastSampleMicros = 0;
  unsigned long nowMicros = micros();

  if (nowMicros - lastSampleMicros >= SAMPLE_INTERVAL_US) {
    lastSampleMicros = nowMicros;

    int dist = readDistance();
    unsigned long nowMs = millis();
    currentDistance = dist;

    updateLapLogic(dist, nowMs);
  }

  // Handle HTTP clients
  server.handleClient();
}
