/*
 * Task 9 - ESP32 PID Line Follower (Arduino framework, C++)
 *
 *  1. Line position : weighted centroid of min-max calibrated analog IR sensors
 *  2. Control       : PID on the centroid error -> differential motor speed
 *  3. Lost line     : steer toward the last side the line was seen for a short
 *                     time, then SAFE STOP (resumes automatically if line returns)
 *  4. Telemetry     : CSV every loop at 115200 baud: S1..S5,error,L,R
 *  5. Validation    : RUN_SAMPLE_TESTS = true runs hard-coded arrays, no motors
 *  6. Wi-Fi monitor : optional phone dashboard (SoftAP) with a latched STOP button
 *
 * WIRING NOTES
 *  - Rightmost sensor is on GPIO36 (ADC1_CH0) instead of GPIO25: GPIO25 is an
 *    ADC2 pin and ADC2 does not work while Wi-Fi is running.
 *  - IN3/IN4 are on GPIO23/22 instead of GPIO12/11: GPIO12 is a boot strapping
 *    pin and GPIO11 is connected to the SPI flash on the ESP32-WROOM-32.
 *
 * PID TUNING NOTES
 *  - Error is in sensor-position units, -2 .. +2 (positive = line on the right).
 *  - KP = 45 : full-scale error (2) gives about 90 PWM of correction.
 *  - KD = 0.18 : damps wobble. Error changes in coarse steps with 5 sensors, so
 *                it is kept small to avoid amplifying noise.
 *  - KI = 0.8, clamped to +-3 error-seconds : only trims slow drift; kept tiny
 *                so it cannot wind up during turns.
 *  - Tune order: KI = KD = 0, raise KP until it just wobbles, back off ~30 %,
 *                add KD until the wobble stops, add KI last (or leave it 0).
 */

#include <Arduino.h>
#include <math.h>
#include <esp_arduino_version.h>
#include <WiFi.h>
#include <WebServer.h>

// --------------------------- USER CONFIGURATION ---------------------------
// true : run hard-coded sample arrays in the Serial Monitor, motors never driven
// false: calibrate and run live on the robot
constexpr bool RUN_SAMPLE_TESTS = false;

// IR sensors, leftmost to rightmost (higher voltage = darker surface = line)
const uint8_t SENSOR_PINS[5] = {34, 35, 32, 33, 36};

// Sensor positions (weights): left negative, right positive
const float SENSOR_POSITIONS[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};

// L298N motor driver pins
const uint8_t PIN_ENA = 14;  // Left motor PWM
const uint8_t PIN_IN1 = 27;  // Left motor direction A
const uint8_t PIN_IN2 = 26;  // Left motor direction B
const uint8_t PIN_ENB = 13;  // Right motor PWM
const uint8_t PIN_IN3 = 23;  // Right motor direction A (remapped from GPIO12)
const uint8_t PIN_IN4 = 22;  // Right motor direction B (remapped from GPIO11)

// Set true if a motor turns backwards when told to go forward
constexpr bool INVERT_LEFT  = false;
constexpr bool INVERT_RIGHT = false;

// PWM configuration
constexpr uint32_t PWM_FREQUENCY_HZ = 15000;
constexpr uint8_t  PWM_RESOLUTION_BITS = 8;   // 0..255
constexpr uint8_t  LEFT_PWM_CHANNEL  = 0;     // used only by Arduino-ESP32 core 2.x
constexpr uint8_t  RIGHT_PWM_CHANNEL = 1;     // used only by Arduino-ESP32 core 2.x

// Timing and signal settings
constexpr uint32_t CALIBRATION_TIME_MS = 4000;
constexpr uint32_t START_DELAY_MS      = 3000;   // time to place the robot after calibration
constexpr uint32_t LOOP_PERIOD_US      = 10000;  // 100 Hz control loop
constexpr uint8_t  SERVER_EVERY_N_LOOPS = 5;     // poll the web server at 20 Hz
constexpr uint16_t MIN_CALIBRATION_SPAN = 100;   // sensor must see both floor and line
constexpr uint16_t NOISE_CUTOFF         = 80;    // ignore small normalised signals (0..1000)
constexpr uint32_t LINE_SUM_THRESHOLD   = 250;   // below this the line is considered lost
constexpr uint32_t LOST_RECOVERY_MS     = 150;   // steer toward last side for this long, then stop

// Speeds (PWM)
constexpr int BASE_SPEED    = 130;
constexpr int MIN_SPEED     = -80;     // limited reverse on the inner wheel for sharp turns
constexpr int MAX_SPEED     = 255;
constexpr float MAX_CORRECTION = 100.0f;

// Initial PID gains: starting values, tune on the real robot
constexpr float KP = 45.0f;
constexpr float KI = 0.8f;
constexpr float KD = 0.18f;
constexpr float INTEGRAL_LIMIT = 3.0f;   // +-3 error-seconds

// ------------------------------- STATE ------------------------------------
uint16_t sensorMin[5];
uint16_t sensorMax[5];
uint16_t rawReadings[5];
uint16_t lineStrength[5];      // normalised 0..1000

float integralError = 0.0f;
float previousError = 0.0f;
bool  pidHasPreviousError = false;
uint32_t previousLoopStartUs = 0;
bool  pwmReady = false;

float    lastError = 0.0f;     // last error while the line was visible
uint32_t lostSinceMs = 0;      // 0 = line currently visible
uint8_t  serverLoopCounter = 0;

// Wi-Fi SoftAP: connect a phone/laptop, then open http://192.168.4.1
const char *WIFI_SSID = "LMS-Robot-ESP32";
const char *WIFI_PASSWORD = "LMSrobot123";   // at least 8 characters
WebServer server(80);
bool wifiDashboardEnabled = false;
volatile bool remoteStopRequested = false;

// Values shown on the dashboard
bool     telemetryLineFound = false;
float    telemetryError = 0.0f;
uint32_t telemetrySignalSum = 0;
int      telemetryLeftPWM = 0;
int      telemetryRightPWM = 0;
bool     calibrationOK = false;

// --------------------------- MOTOR FUNCTIONS -----------------------------
void writeMotorPwm(uint8_t pin, uint8_t channel, uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcWrite(pin, duty);        // core 3.x addresses LEDC by pin
#else
  (void)pin;
  ledcWrite(channel, duty);    // core 2.x addresses LEDC by channel
#endif
}

bool setupMotorDriver() {
  const uint8_t outs[6] = {PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4, PIN_ENA, PIN_ENB};
  for (uint8_t i = 0; i < 6; ++i) {
    pinMode(outs[i], OUTPUT);
    digitalWrite(outs[i], LOW);
  }

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  const bool leftOK  = ledcAttach(PIN_ENA, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  const bool rightOK = ledcAttach(PIN_ENB, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  pwmReady = leftOK && rightOK;
#else
  ledcSetup(LEFT_PWM_CHANNEL,  PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcSetup(RIGHT_PWM_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(PIN_ENA, LEFT_PWM_CHANNEL);
  ledcAttachPin(PIN_ENB, RIGHT_PWM_CHANNEL);
  pwmReady = true;
#endif
  return pwmReady;
}

void stopMotors() {
  if (pwmReady) {
    writeMotorPwm(PIN_ENA, LEFT_PWM_CHANNEL, 0);
    writeMotorPwm(PIN_ENB, RIGHT_PWM_CHANNEL, 0);
  }
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
}

// One motor, signed speed: >0 forward, <0 reverse, 0 stop
void setMotor(uint8_t inA, uint8_t inB, uint8_t enPin, uint8_t channel, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0)      { digitalWrite(inA, HIGH); digitalWrite(inB, LOW);  }
  else if (speed < 0) { digitalWrite(inA, LOW);  digitalWrite(inB, HIGH); }
  else                { digitalWrite(inA, LOW);  digitalWrite(inB, LOW);  }
  writeMotorPwm(enPin, channel, (uint8_t)abs(speed));
}

void driveMotors(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed,  MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);
  setMotor(PIN_IN1, PIN_IN2, PIN_ENA, LEFT_PWM_CHANNEL,  INVERT_LEFT  ? -leftSpeed  : leftSpeed);
  setMotor(PIN_IN3, PIN_IN4, PIN_ENB, RIGHT_PWM_CHANNEL, INVERT_RIGHT ? -rightSpeed : rightSpeed);
}

// ----------------------------- CALIBRATION -------------------------------
void configureSensors() {
  analogReadResolution(12);   // 0..4095
  for (int i = 0; i < 5; ++i) {
    pinMode(SENSOR_PINS[i], INPUT);
    analogSetPinAttenuation(SENSOR_PINS[i], ADC_11db);
  }
}

bool calibrateSensors() {
  for (int i = 0; i < 5; ++i) {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }

  Serial.println();
  Serial.println(F("CALIBRATION: for 4 seconds sweep the array so EVERY sensor sees both the floor and the line."));
  Serial.println(F("Motors stay off during calibration."));

  const uint32_t startMs = millis();
  while ((millis() - startMs) < CALIBRATION_TIME_MS) {
    if (wifiDashboardEnabled) server.handleClient();
    for (int i = 0; i < 5; ++i) {
      const uint16_t value = (uint16_t)analogRead(SENSOR_PINS[i]);
      if (value < sensorMin[i]) sensorMin[i] = value;
      if (value > sensorMax[i]) sensorMax[i] = value;
    }
    delay(5);
  }

  bool allSensorsValid = true;
  Serial.println(F("Calibration ranges (min..max):"));
  for (int i = 0; i < 5; ++i) {
    Serial.printf("S%d GPIO%d: %u..%u, span=%u\n",
                  i + 1, SENSOR_PINS[i], sensorMin[i], sensorMax[i],
                  (uint16_t)(sensorMax[i] - sensorMin[i]));
    if ((sensorMax[i] - sensorMin[i]) < MIN_CALIBRATION_SPAN) allSensorsValid = false;
  }
  return allSensorsValid;
}

void readAndNormalizeSensors(uint16_t raw[5], uint16_t strength[5]) {
  for (int i = 0; i < 5; ++i) {
    raw[i] = (uint16_t)analogRead(SENSOR_PINS[i]);
    const int span = (int)sensorMax[i] - (int)sensorMin[i];
    if (span <= 0) { strength[i] = 0; continue; }

    // Higher voltage = darker surface, so a larger value = more line evidence
    long scaled = ((long)raw[i] - (long)sensorMin[i]) * 1000L / span;
    strength[i] = (uint16_t)constrain(scaled, 0L, 1000L);
  }
}

// --------------------------- CENTROID + PID -------------------------------
// Returns false when there is not enough signal to say a line is present.
// position is in [-2,+2]: negative = line on the left, positive = line on the right.
bool estimateLinePosition(const uint16_t strength[5], float &position, uint32_t &signalSum) {
  float weightedSum = 0.0f;
  signalSum = 0;

  for (int i = 0; i < 5; ++i) {
    uint16_t signal = strength[i];
    if (signal < NOISE_CUTOFF) signal = 0;
    signalSum += signal;
    weightedSum += SENSOR_POSITIONS[i] * (float)signal;
  }

  if (signalSum < LINE_SUM_THRESHOLD) {   // also protects against division by zero
    position = 0.0f;
    return false;
  }
  position = weightedSum / (float)signalSum;   // weighted centroid
  return true;
}

void resetPID() {
  integralError = 0.0f;
  previousError = 0.0f;
  pidHasPreviousError = false;
}

float computePID(float error, float dtSeconds) {
  if (dtSeconds <= 0.0f || dtSeconds > 0.1f) dtSeconds = LOOP_PERIOD_US / 1000000.0f;

  integralError += error * dtSeconds;
  integralError = constrain(integralError, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float derivative = 0.0f;
  if (pidHasPreviousError) derivative = (error - previousError) / dtSeconds;
  previousError = error;
  pidHasPreviousError = true;

  const float correction = KP * error + KI * integralError + KD * derivative;
  return constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
}

// Positive correction (line on the right) -> left wheel faster, right wheel slower
void applyCorrection(float correction, int &leftSpeed, int &rightSpeed) {
  leftSpeed  = constrain((int)lroundf(BASE_SPEED + correction), MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain((int)lroundf(BASE_SPEED - correction), MIN_SPEED, MAX_SPEED);
}

// CSV telemetry, one line per loop: S1,S2,S3,S4,S5,error,L,R
void printTelemetry(const uint16_t raw[5], bool lineFound, float error, int leftSpeed, int rightSpeed) {
  for (int i = 0; i < 5; ++i) { Serial.print(raw[i]); Serial.print(','); }
  if (lineFound) Serial.print(error, 3);
  else           Serial.print(F("LOST"));
  Serial.printf(",%d,%d\n", leftSpeed, rightSpeed);
}

// ----------------------------- WI-FI DASHBOARD ----------------------------
// Telemetry viewer plus a latched STOP button. PID always runs on the robot.
const char DASHBOARD_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>LMS Robot Monitor</title>
<style>
body{font-family:system-ui,Arial,sans-serif;max-width:760px;margin:20px auto;padding:0 14px;background:#f4f6f8;color:#18212b}
main{background:white;padding:18px;border-radius:14px;box-shadow:0 2px 12px #0001}
h1{font-size:1.4rem;margin-top:0}.muted{color:#56616e;font-size:.9rem}.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}
.card{background:#f0f3f6;padding:12px;border-radius:10px}.value{font-size:1.35rem;font-weight:700;overflow-wrap:anywhere}
button{border:0;border-radius:9px;padding:12px 18px;font-size:1rem;font-weight:700;margin:8px 8px 8px 0;cursor:pointer}
.stop{background:#b42318;color:#fff}.run{background:#176b3a;color:#fff}#sensors{font-family:ui-monospace,monospace;word-break:break-word}
</style></head><body><main><h1>LMS Robot - Wi-Fi Monitor</h1>
<p class="muted">Live telemetry from the ESP32. Control and PID calculations stay on the robot.</p>
<div class="grid">
<div class="card"><div class="muted">Mode</div><div class="value" id="mode">Connecting...</div></div>
<div class="card"><div class="muted">Line position / error</div><div class="value" id="error">-</div></div>
<div class="card"><div class="muted">Left motor</div><div class="value" id="left">0</div></div>
<div class="card"><div class="muted">Right motor</div><div class="value" id="right">0</div></div>
</div>
<h3>IR sensor readings (left to right)</h3><div id="sensors">Waiting for data...</div>
<p class="muted" id="signal"></p>
<button class="stop" onclick="sendCommand('/api/stop')">EMERGENCY STOP</button>
<button class="run" onclick="sendCommand('/api/run')">RESUME PID</button>
<p class="muted" id="message">Connect to Wi-Fi SSID LMS-Robot-ESP32, then browse to http://192.168.4.1</p>
</main><script>
async function refresh(){try{const r=await fetch('/api/telemetry',{cache:'no-store'});const d=await r.json();
 document.getElementById('mode').textContent=d.stopped?'STOPPED':(d.lineFound?'TRACKING LINE':'LINE LOST');
 document.getElementById('error').textContent=d.lineFound?Number(d.error).toFixed(3):'LOST';
 document.getElementById('left').textContent=d.leftPWM;document.getElementById('right').textContent=d.rightPWM;
 document.getElementById('sensors').textContent='Raw: ['+d.raw.join(', ')+'] | Normalized: ['+d.norm.join(', ')+']';
 document.getElementById('signal').textContent='Line signal sum: '+d.signalSum+' | Calibration: '+(d.calibrationOK?'OK':'not confirmed');
 }catch(e){document.getElementById('mode').textContent='Reconnecting...';}}
async function sendCommand(path){try{const r=await fetch(path,{method:'POST'});const d=await r.json();document.getElementById('message').textContent=d.message;await refresh();}catch(e){document.getElementById('message').textContent='Command failed; check Wi-Fi connection.';}}
refresh();setInterval(refresh,250);
</script></body></html>
)HTML";

void handleTelemetryRequest() {
  String json;
  json.reserve(200);
  json += "{\"raw\":[";
  for (int i = 0; i < 5; ++i) { if (i) json += ','; json += String(rawReadings[i]); }
  json += "],\"norm\":[";
  for (int i = 0; i < 5; ++i) { if (i) json += ','; json += String(lineStrength[i]); }
  json += "],\"lineFound\":";
  json += telemetryLineFound ? "true" : "false";
  json += ",\"error\":" + String(telemetryError, 3);
  json += ",\"signalSum\":" + String(telemetrySignalSum);
  json += ",\"leftPWM\":" + String(telemetryLeftPWM);
  json += ",\"rightPWM\":" + String(telemetryRightPWM);
  json += ",\"stopped\":";
  json += remoteStopRequested ? "true" : "false";
  json += ",\"calibrationOK\":";
  json += calibrationOK ? "true" : "false";
  json += '}';
  server.send(200, "application/json", json);
}

void startWiFiDashboard() {
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  if (!WiFi.softAP(WIFI_SSID, WIFI_PASSWORD)) {
    Serial.println(F("WARNING: Wi-Fi access point failed to start; line following still works via Serial."));
    return;
  }

  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", DASHBOARD_HTML); });
  server.on("/api/telemetry", HTTP_GET, handleTelemetryRequest);
  server.on("/api/stop", HTTP_POST, []() {
    remoteStopRequested = true;
    resetPID();
    stopMotors();
    telemetryLeftPWM = 0;
    telemetryRightPWM = 0;
    server.send(200, "application/json", "{\"ok\":true,\"message\":\"Emergency stop latched.\"}");
  });
  server.on("/api/run", HTTP_POST, []() {
    remoteStopRequested = false;
    resetPID();
    server.send(200, "application/json", "{\"ok\":true,\"message\":\"PID enabled. Robot moves only if the line is detected.\"}");
  });
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();
  wifiDashboardEnabled = true;

  Serial.println();
  Serial.println(F("Wi-Fi dashboard ready:"));
  Serial.printf("  SSID: %s\n  Password: %s\n  URL: http://%s\n",
                WIFI_SSID, WIFI_PASSWORD, WiFi.softAPIP().toString().c_str());
}

// ----------------------------- SAMPLE TESTS -------------------------------
void runOneSample(const char *name, const uint16_t sample[5]) {
  float error = 0.0f;
  uint32_t signalSum = 0;
  resetPID();   // each example is independent

  Serial.printf("\nTEST %s: [", name);
  for (int i = 0; i < 5; ++i) { if (i) Serial.print(','); Serial.print(sample[i]); }
  Serial.print(']');

  if (!estimateLinePosition(sample, error, signalSum)) {
    Serial.printf(" -> LOST LINE (signal=%lu): recovery steering, then safe stop L=0 R=0\n",
                  (unsigned long)signalSum);
    return;
  }

  const float correction = computePID(error, LOOP_PERIOD_US / 1000000.0f);
  int left, right;
  applyCorrection(correction, left, right);
  Serial.printf(" -> error=%.3f, PID correction=%.2f, proposed L=%d R=%d (not applied)\n",
                error, correction, left, right);
}

void runSampleTests() {
  // Already-normalised strengths (0..1000), leftmost to rightmost
  const uint16_t centered[5]  = {0, 0, 1000, 0, 0};
  const uint16_t lineLeft[5]  = {900, 650, 100, 0, 0};
  const uint16_t lineRight[5] = {0, 0, 120, 700, 900};
  const uint16_t between[5]   = {0, 0, 800, 800, 0};     // line between S3 and S4
  const uint16_t noLine[5]    = {20, 30, 10, 25, 15};

  Serial.println(F("\n=== HARD-CODED SENSOR TESTS (motors are NOT driven) ==="));
  runOneSample("center", centered);
  runOneSample("left-biased", lineLeft);
  runOneSample("right-biased", lineRight);
  runOneSample("between S3 and S4", between);
  runOneSample("lost-line", noLine);
  Serial.println(F("\nSample tests complete. Set RUN_SAMPLE_TESTS=false for live operation."));
}

// -------------------------------- SETUP -----------------------------------
void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println(F("Task 9 - ESP32 PID line follower"));

  configureSensors();
  if (!setupMotorDriver()) {
    Serial.println(F("ERROR: PWM setup failed. Motors remain stopped."));
    while (true) delay(1000);
  }
  stopMotors();

  if (RUN_SAMPLE_TESTS) {
    runSampleTests();
    stopMotors();
    while (true) delay(1000);
  }

  startWiFiDashboard();
  delay(1000);
  calibrationOK = calibrateSensors();
  if (!calibrationOK) {
    Serial.println(F("ERROR: a sensor span is too small. Motors stopped."));
    Serial.println(F("Reset and sweep every sensor over both the floor and the line."));
    stopMotors();
    while (true) {
      if (wifiDashboardEnabled) server.handleClient();
      delay(10);
    }
  }

  Serial.println(F("Calibration OK. Place the robot on the line..."));
  for (int s = START_DELAY_MS / 1000; s > 0; --s) {
    Serial.printf("Starting in %d...\n", s);
    const uint32_t t0 = millis();
    while (millis() - t0 < 1000) {
      if (wifiDashboardEnabled) server.handleClient();
      delay(5);
    }
  }
  Serial.println(F("S1,S2,S3,S4,S5,error,L,R"));
  previousLoopStartUs = micros();
}

// --------------------------------- LOOP -----------------------------------
void loop() {
  const uint32_t loopStartUs = micros();
  if (wifiDashboardEnabled && ++serverLoopCounter >= SERVER_EVERY_N_LOOPS) {
    serverLoopCounter = 0;
    server.handleClient();
  }

  const float dtSeconds = (loopStartUs - previousLoopStartUs) / 1000000.0f;
  previousLoopStartUs = loopStartUs;

  readAndNormalizeSensors(rawReadings, lineStrength);

  float error = 0.0f;
  uint32_t signalSum = 0;
  const bool lineFound = estimateLinePosition(lineStrength, error, signalSum);
  telemetryLineFound = lineFound;
  telemetryError = error;
  telemetrySignalSum = signalSum;

  int leftSpeed = 0, rightSpeed = 0;

  if (remoteStopRequested) {
    // Wi-Fi STOP is latched until RESUME is pressed; sensing and telemetry stay alive
    resetPID();
    stopMotors();
  } else if (lineFound) {
    lostSinceMs = 0;
    lastError = error;
    applyCorrection(computePID(error, dtSeconds), leftSpeed, rightSpeed);
    driveMotors(leftSpeed, rightSpeed);
  } else {
    if (lostSinceMs == 0) lostSinceMs = millis();
    if (millis() - lostSinceMs < LOST_RECOVERY_MS) {
      // Short recovery: steer hard toward the side where the line was last seen
      const float pretendError = (lastError >= 0.0f) ? 2.0f : -2.0f;
      applyCorrection(computePID(pretendError, dtSeconds), leftSpeed, rightSpeed);
      driveMotors(leftSpeed, rightSpeed);
    } else {
      // Line lost for too long: safe stop (resumes automatically if the line returns)
      resetPID();
      stopMotors();
    }
  }

  telemetryLeftPWM = leftSpeed;
  telemetryRightPWM = rightSpeed;
  printTelemetry(rawReadings, lineFound, error, leftSpeed, rightSpeed);

  const uint32_t elapsedUs = micros() - loopStartUs;
  if (elapsedUs < LOOP_PERIOD_US) delayMicroseconds(LOOP_PERIOD_US - elapsedUs);
}
