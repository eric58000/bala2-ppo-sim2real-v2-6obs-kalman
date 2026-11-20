/*
  BALA2 Fire - Real-world test of the NEW six-observation PPO actor, without output correction.
  Actor: replace actor.h with the version exported after the latest training run.

  Button A: engage the automatic controller, only when close to upright.
  Button B: stop immediately and save the log.
  Always start disarmed; never restart automatically after a fall.
  USB commands while stopped: D (CSV log), E (erase), H (help).

  WARNING: this firmware actually drives the motors.
  Motor directions and real-world policy robustness have NOT yet been validated.
  Perform the first test with fall protection.
*/
#include <Arduino.h>
#include <M5Unified.h>
#include <SPIFFS.h>
#include <math.h>
#include <esp_heap_caps.h>
#include "actor.h"
#include "bala.h"
#include "KalmanPitch.h"

static_assert(ACTOR_OBS_SIZE == 6, "actor.h doit avoir 6 observations");
static_assert(ACTOR_ACTION_SIZE == 2, "actor.h doit avoir 2 actions");

constexpr float PITCH_OFFSET = 0.0f; // Do NOT apply +3.6 degrees without a mechanical reference
constexpr float DT_NOMINAL = 0.005f;
constexpr uint32_t PERIOD_US = 5000; // 200 Hz
constexpr uint32_t MAX_IMU_DT_US = 30000;
constexpr float GYRO_RAD = PI / 180.0f;
constexpr float ENC_TICKS_TO_RADS = 2.0f * PI / 414.0f;
constexpr uint8_t VELOCITY_WINDOW_STEPS = 5; // 25 ms
constexpr int32_t ENC_DIR_LEFT = 1, ENC_DIR_RIGHT = 1;
constexpr int16_t MOTOR_DIR_LEFT = -1, MOTOR_DIR_RIGHT = -1;
constexpr float MOTOR_SCALE = 1023.0f;
constexpr float MOTOR_BOOST = 1.0f;

// NO correction to PPO predictions. Only physical output saturation to [-1,1].
// Drift monitoring (wide safety threshold, without modifying the policy).
// 0 disables the drift-related stop. The thresholds below are in radians.
constexpr float MAX_WHEEL_DIFFERENCE_RAD = 6.0f;
constexpr float MAX_MEAN_POSITION_RAD = 15.0f;
constexpr float ACCEL_VALID_MIN_G = 0.80f;
constexpr float ACCEL_VALID_MAX_G = 1.20f;
constexpr float START_ANGLE_LIMIT = 0.15f; // +/-8.6 degrees at startup
constexpr float FALL_ANGLE_LIMIT = 0.79f;  // +/-45 degrees (Kalman)
constexpr float ACCEL_HARD_LIMIT = 1.04719755f; // +/-60 degrees from accelerometer
constexpr uint8_t ACCEL_HARD_CONSECUTIVE = 3; // ~15 ms; rejects isolated spikes
constexpr uint16_t GYRO_CAL_SAMPLES = 400;
constexpr uint32_t LOG_PERIOD_MS = 100;
constexpr size_t LOG_CAPACITY_PSRAM = 1200; // 120 s at 10 Hz with PSRAM
constexpr size_t LOG_CAPACITY_FALLBACK = 250; // 25 s at 10 Hz without PSRAM
constexpr char LOG_FILE[] = "/balance_log.csv";

struct LogSample {
  uint32_t ms;
  float pitch, rate, vL, vR, pL, pR, aL, aR;
  float pMean, pDiff, vMean, vDiff;
  int16_t mL, mR;
  uint16_t missed;
  float gyro, bias, accelAngle, accelNorm, dt;
  uint8_t event; // 0=periodic, 1=stop
};
// Do not reserve 100+ KB in the ESP32 .dram0.bss section.
static LogSample* logBuffer = nullptr;
static size_t logCapacity = 0;
static size_t logCount = 0, logNext = 0;
static bool flashReady = false, running = false, imuReady = false;
static uint32_t trialStartedMs = 0, lastLogMs = 0, imuPreviousUs = 0;
static uint32_t nextStepUs = 0, missedDeadlines = 0;
static uint8_t accelHardCount = 0;
static float pitch = 0, rate = 0, rawGyro = 0, accelPitch = 0, accelNorm = 1;
static float measuredDt = DT_NOMINAL, startupGyroBias = 0;
static float vL = 0, vR = 0, pL = 0, pR = 0, aL = 0, aR = 0;
static float pMean = 0, pDiff = 0, vMean = 0, vDiff = 0;
static int16_t mL = 0, mR = 0;
static int32_t encHistoryL[VELOCITY_WINDOW_STEPS] = {};
static int32_t encHistoryR[VELOCITY_WINDOW_STEPS] = {};
static uint8_t histIndex = 0, histCount = 0;
static Bala bala;
static KalmanPitch kalman;

void showStatus(const char* title, const char* detail) {
  // Never refresh the display during the 200 Hz control loop.
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(8, 30);
  M5.Display.println(title);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(8, 88);
  M5.Display.println(detail);
}

void resetVelocity() {
  memset(encHistoryL, 0, sizeof(encHistoryL));
  memset(encHistoryR, 0, sizeof(encHistoryR));
  histCount = histIndex = 0;
}

void wheelVelocity(int32_t eL, int32_t eR) {
  const int32_t oldL = encHistoryL[histIndex];
  const int32_t oldR = encHistoryR[histIndex];
  encHistoryL[histIndex] = eL;
  encHistoryR[histIndex] = eR;
  histIndex = (histIndex + 1) % VELOCITY_WINDOW_STEPS;
  if (histCount < VELOCITY_WINDOW_STEPS) ++histCount;
  const float elapsed = histCount * DT_NOMINAL;
  vL = float(eL - oldL) * ENC_TICKS_TO_RADS / elapsed;
  vR = float(eR - oldR) * ENC_TICKS_TO_RADS / elapsed;
}

void appendLog(uint8_t eventCode) {
  if (!logBuffer || logCapacity == 0) return;
  LogSample &s = logBuffer[logNext];
  s.ms = millis() - trialStartedMs;
  s.pitch = pitch + PITCH_OFFSET;
  s.rate = rate;
  s.vL = vL; s.vR = vR; s.pL = pL; s.pR = pR;
  s.aL = aL; s.aR = aR; s.mL = mL; s.mR = mR;
  s.pMean = pMean; s.pDiff = pDiff;
  s.vMean = vMean; s.vDiff = vDiff;
  s.missed = (uint16_t)(missedDeadlines > 65535 ? 65535 : missedDeadlines);
  s.gyro = rawGyro; s.bias = kalman.bias;
  s.accelAngle = accelPitch; s.accelNorm = accelNorm;
  s.dt = measuredDt; s.event = eventCode;
  logNext = (logNext + 1) % logCapacity;
  if (logCount < logCapacity) ++logCount;
}

void saveLog(const char* stopReason) {
  // Always called with motors stopped; no SPIFFS access while running.
  if (!flashReady) {
    showStatus("STOP / SAFE", stopReason);
    Serial.println("SPIFFS indisponible: journal non persistant");
    return;
  }
  if (!logBuffer || logCapacity == 0 || logCount == 0) {
    showStatus("STOP / NO LOG", stopReason);
    return;
  }
  showStatus("SAVING...", stopReason);
  File f = SPIFFS.open(LOG_FILE, "w"); // overwrites the previous log
  if (!f) {
    showStatus("SAVE FAILED", stopReason);
    return;
  }
  f.print("ms,pitch,pitch_rate,vL,vR,pL,pR,aL,aR,mL,mR,missed_deadlines,gyro_raw,gyro_bias,accel_pitch,accel_norm_g,dt_s,p_mean,p_diff,v_mean,v_diff,event,stop_reason\n");
  const size_t first = (logNext + logCapacity - logCount) % logCapacity;
  for (size_t k = 0; k < logCount; ++k) {
    const LogSample &s = logBuffer[(first + k) % logCapacity];
    f.printf("%lu,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%d,%d,%u,%.5f,%.5f,%.5f,%.5f,%.6f,%.5f,%.5f,%.5f,%.5f,%s,%s\n",
      (unsigned long)s.ms, s.pitch, s.rate, s.vL, s.vR, s.pL, s.pR,
      s.aL, s.aR, (int)s.mL, (int)s.mR, (unsigned)s.missed,
      s.gyro, s.bias, s.accelAngle, s.accelNorm, s.dt,
      s.pMean, s.pDiff, s.vMean, s.vDiff,
      s.event ? "STOP" : "PERIODIC", s.event ? stopReason : "");
  }
  f.flush();
  f.close();
  Serial.printf("STOP_REASON=%s; SAMPLES=%u\n", stopReason, (unsigned)logCount);
  showStatus("STOP / SAFE", stopReason);
}

void haltRobot(const char* reason) {
  bala.SetSpeed(0, 0);
  mL = mR = 0;
  if (!running) return;
  running = false;
  appendLog(1); // terminal event, even between two 100 ms logging intervals
  saveLog(reason);
}

void dumpLog() {
  if (!flashReady || !SPIFFS.exists(LOG_FILE)) {
    Serial.println("NO_LOG"); return;
  }
  File f = SPIFFS.open(LOG_FILE, "r");
  if (!f) { Serial.println("LOG_READ_ERROR"); return; }
  Serial.println("BEGIN_CSV");
  uint8_t b[128];
  while (f.available()) {
    size_t n = f.read(b, sizeof(b));
    if (!n) break;
    Serial.write(b, n);
  }
  f.close();
  Serial.println("END_CSV");
}

void calibrateGyroAtBoot() {
  // Internal bias calibration, not a user-operated calibration mode.
  // Perfect upright alignment is NOT required; remaining motionless is.
  double sum = 0.0, sq = 0.0;
  for (uint16_t i = 0; i < GYRO_CAL_SAMPLES; ++i) {
    M5.Imu.update();
    const auto imu = M5.Imu.getImuData();
    const float gyro = -imu.gyro.x * GYRO_RAD;
    if (!isfinite(gyro)) { imuReady = false; return; }
    sum += gyro;
    sq += double(gyro) * gyro;
    delay(5);
  }
  const float mean = float(sum / GYRO_CAL_SAMPLES);
  const float stddev = sqrtf(float(fmax(0.0, sq / GYRO_CAL_SAMPLES - double(mean) * mean)));
  imuReady = stddev < 0.10f && fabsf(mean) < 0.5f;
  startupGyroBias = mean;
  Serial.printf("GYRO_BIAS=%.5f rad/s, sigma=%.5f, valid=%d\n",
                startupGyroBias, stddev, (int)imuReady);
}

void startBalance() {
  if (!imuReady) { showStatus("IMU ERROR", "Restart robot still"); return; }
  bala.SetSpeed(0, 0);
  M5.Imu.update();
  const auto d = M5.Imu.getImuData();
  const float startAccelAngle = -atan2f(d.accel.y, d.accel.z);
  const float norm = sqrtf(d.accel.x*d.accel.x + d.accel.y*d.accel.y + d.accel.z*d.accel.z);
  if (!isfinite(startAccelAngle) || !isfinite(norm) ||
      norm < ACCEL_VALID_MIN_G || norm > ACCEL_VALID_MAX_G ||
      d.accel.z <= 0.0f || fabsf(startAccelAngle + PITCH_OFFSET) > START_ANGLE_LIMIT) {
    showStatus("NOT UPRIGHT", "Reposition; press A");
    return;
  }
  kalman.reset(startAccelAngle, startupGyroBias);
  kalman.biasAdaptLimit = 0.26f;
  pitch = startAccelAngle;
  rate = rawGyro = 0.0f;
  accelPitch = startAccelAngle;
  accelNorm = norm;
  vL = vR = pL = pR = aL = aR = 0.0f;
  pMean = pDiff = vMean = vDiff = 0.0f;
  mL = mR = 0;
  missedDeadlines = 0;
  accelHardCount = 0;
  logCount = logNext = 0;
  bala.ClearEncoder();
  resetVelocity();
  showStatus("PPO ACTIVE", "B: emergency stop");
  // Initialize timing AFTER I2C and display setup (avoid an incorrect initial dt)
  M5.Imu.update();
  imuPreviousUs = micros();
  nextStepUs = imuPreviousUs + PERIOD_US;
  trialStartedMs = lastLogMs = millis();
  running = true;
}

void handleSerialStopped() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'D' || c == 'd') dumpLog();
    else if (c == 'E' || c == 'e') {
      if (flashReady && SPIFFS.exists(LOG_FILE)) {
        Serial.println(SPIFFS.remove(LOG_FILE) ? "LOG_ERASED" : "ERASE_FAILED");
      } else Serial.println("NO_LOG");
    }
    else if (c == 'H' || c == 'h') {
      Serial.println("A: balance | B: stop | X: stop | D: CSV | E: erase CSV");
      Serial.println("Power-up: motors OFF. Logs auto-overwritten after each stop.");
      Serial.printf("PPO_ONLY=1; MAX_DIFF=%.2f; MAX_MEAN=%.2f\n",
                    MAX_WHEEL_DIFFERENCE_RAD, MAX_MEAN_POSITION_RAD);
    }
  }
}

void initLogBuffer() {
  // A PSRAM allocation does not increase the .dram0.bss section.
  logCapacity = LOG_CAPACITY_PSRAM;
  logBuffer = static_cast<LogSample*>(
      heap_caps_malloc(logCapacity * sizeof(LogSample), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!logBuffer) {
    // If PSRAM is unavailable, keep a shorter log in internal RAM.
    logCapacity = LOG_CAPACITY_FALLBACK;
    logBuffer = static_cast<LogSample*>(
        heap_caps_malloc(logCapacity * sizeof(LogSample), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  }
  if (!logBuffer) logCapacity = 0;
  logCount = logNext = 0;
  Serial.printf("LOG_BUFFER=%s, capacity=%u, bytes=%u\n",
                logBuffer ? "READY" : "OFF",
                (unsigned)logCapacity,
                (unsigned)(logCapacity * sizeof(LogSample)));
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(0);
  Serial.begin(115200);
  initLogBuffer();
  bala.SetSpeed(0, 0);
  showStatus("PPO NEW MODEL", "Initializing IMU...");
  if (!M5.Imu.loadOffsetFromNVS()) {
    imuReady = false;
    showStatus("IMU NVS ERROR", "Previous calibration needed");
    Serial.println("IMU_NVS_ERROR");
  } else {
    showStatus("PPO NEW MODEL", "Keep still (2 sec)");
    calibrateGyroAtBoot();
  }
  flashReady = SPIFFS.begin(false);
  if (!imuReady) showStatus("IMU ERROR", "Check IMU; restart");
  else showStatus("READY / OFF", "A: balance | B: stop");
  Serial.println("CONTROL=PPO_ONLY; OBS=6; OUTPUT=2; NO_ADDITIVE_CORRECTION");
  Serial.printf("SPIFFS=%s\n", flashReady ? "READY" : "OFF (balancing allowed)");
}

void loop() {
  if (!running) {
    M5.update();
    // Keep motors stopped continuously while idle.
    bala.SetSpeed(0, 0);
    handleSerialStopped();
    if (M5.BtnA.wasPressed()) startBalance();
    delay(10);
    return;
  }

  const uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextStepUs) < 0) {
    // Brief wait without I2C to maintain control-loop timing.
    return;
  }
  // If a period is missed, do not accumulate multiple delayed commands.
  if ((uint32_t)(nowUs - nextStepUs) >= PERIOD_US) ++missedDeadlines;
  nextStepUs = nowUs + PERIOD_US;
  const uint32_t cycleStartedUs = micros();
  M5.update();
  if (M5.BtnB.wasPressed()) { haltRobot("BUTTON_B"); return; }
  // USB command X is an additional stop command, not a motor action.
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'X' || c == 'x') { haltRobot("SERIAL_X"); return; }
  }

  M5.Imu.update();
  const auto imu = M5.Imu.getImuData();
  const uint32_t imuNowUs = micros();
  const uint32_t dtUs = imuNowUs - imuPreviousUs;
  imuPreviousUs = imuNowUs;
  measuredDt = dtUs * 1e-6f;
  rawGyro = -imu.gyro.x * GYRO_RAD;
  accelPitch = -atan2f(imu.accel.y, imu.accel.z);
  accelNorm = sqrtf(imu.accel.x*imu.accel.x +
                    imu.accel.y*imu.accel.y + imu.accel.z*imu.accel.z);
  if (!isfinite(rawGyro) || !isfinite(accelPitch) || !isfinite(accelNorm) ||
      dtUs == 0 || dtUs > MAX_IMU_DT_US) {
    haltRobot("IMU_OR_TIMING"); return;
  }

  const bool accelValid = accelNorm >= ACCEL_VALID_MIN_G && accelNorm <= ACCEL_VALID_MAX_G;
  kalman.update(rawGyro, accelPitch, measuredDt, accelValid);
  pitch = kalman.angle;
  rate = kalman.rate;
  if (!isfinite(pitch) || !isfinite(rate)) { haltRobot("IMU_INVALID"); return; }
  if (fabsf(pitch + PITCH_OFFSET) > FALL_ANGLE_LIMIT) {
    haltRobot("TIP_KALMAN"); return;
  }
  // Require three consecutive cycles to avoid stopping on a single
  // accelerometer reading, while retaining immediate Kalman safety shutdown.
  if (accelValid && fabsf(accelPitch) > ACCEL_HARD_LIMIT) {
    if (++accelHardCount >= ACCEL_HARD_CONSECUTIVE) {
      haltRobot("TIP_ACCEL"); return;
    }
  } else accelHardCount = 0;

  int32_t eL, eR;
  bala.GetEncoder(&eL, &eR);
  eL *= ENC_DIR_LEFT;
  eR *= ENC_DIR_RIGHT;
  wheelVelocity(eL, eR);
  pL = float(eL) * ENC_TICKS_TO_RADS;
  pR = float(eR) * ENC_TICKS_TO_RADS;
  pMean = 0.5f * (pL + pR);
  pDiff = pR - pL;
  vMean = 0.5f * (vL + vR);
  vDiff = vR - vL;
  if (!isfinite(pMean) || !isfinite(pDiff) ||
      !isfinite(vMean) || !isfinite(vDiff)) {
    haltRobot("ENCODER_INVALID"); return;
  }
  if ((MAX_WHEEL_DIFFERENCE_RAD > 0.0f &&
       fabsf(pDiff) > MAX_WHEEL_DIFFERENCE_RAD) ||
      (MAX_MEAN_POSITION_RAD > 0.0f &&
       fabsf(pMean) > MAX_MEAN_POSITION_RAD)) {
    haltRobot("WHEEL_RUNAWAY"); return;
  }

  const float obs[ACTOR_OBS_SIZE] = {
    pitch + PITCH_OFFSET, rate, vL, vR, pL, pR
  };
  float actions[ACTOR_ACTION_SIZE];
  actor_forward(obs, actions);
  if (!isfinite(actions[0]) || !isfinite(actions[1])) {
    haltRobot("ACTOR_INVALID"); return;
  }
  // No differential correction, no PID, no action filtering.
  // Saturate only at the physical motor command limits.
  aL = constrain(actions[0], -1.0f, 1.0f);
  aR = constrain(actions[1], -1.0f, 1.0f);
  mL = int16_t(MOTOR_DIR_LEFT * (aL * MOTOR_SCALE * MOTOR_BOOST));
  mR = int16_t(MOTOR_DIR_RIGHT * (aR * MOTOR_SCALE * MOTOR_BOOST));
  mL = constrain(mL, (int16_t)-1023, (int16_t)1023);
  mR = constrain(mR, (int16_t)-1023, (int16_t)1023);
  bala.SetSpeed(mL, mR);

  const uint32_t ms = millis();
  if (ms - lastLogMs >= LOG_PERIOD_MS) {
    lastLogMs = ms;
    appendLog(0);
  }
  if ((uint32_t)(micros() - cycleStartedUs) > PERIOD_US) ++missedDeadlines;
}
