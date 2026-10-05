/*
  ============================================================
  ESP32 GESTURE GLOVE — BLE HID GAMEPAD
  ============================================================
  A wearable glove that reads hand orientation (pitch/roll/yaw)
  from an MPU6050 and broadcasts it as a real Bluetooth LE HID
  Gamepad — Windows / Android see it as a normal game controller,
  no companion app or PC-side software required.

  FEATURES
    - DMP-based orientation sensing (stable, low-drift pitch/roll)
    - Auto neutral-pose calibration on boot + manual re-center
      (long-press, see button map below)
    - Exponential smoothing + windowed stillness gate (kills jitter
      and residual drift while the hand is genuinely still)
    - Expo-curve + slew-rate-limited axis output (gentle near center,
      full range only on deliberate tilt — feels like a real analog
      stick rather than a twitchy 1:1 mapping)
    - Debounced tactile buttons, tap/hold/deep-hold tiered logic on
      one button (jump / mode switch / re-center)
    - OLED status display (mode, BLE state, battery, live P/R/Y)
    - Haptic feedback via vibration motor on key events

  HARDWARE / WIRING
    MPU6050  -> SDA=21, SCL=22 (I2C bus)
    OLED     -> SDA=21, SCL=22 (shares the same I2C bus, addr 0x3C)
    Button (Index finger)  -> GPIO 27  (tap = Jump | hold 0.6-1.8s = Mode switch | hold 1.8s+ = Re-center pose)
    Button (Middle finger) -> GPIO 26  (Shoot)
    Button (Ring finger)   -> GPIO 32  (Reload)
    Button (Pinky finger)  -> GPIO 33  (Melee)
    Vibration motor        -> GPIO 25  (via NPN transistor, e.g. 2N2222 + base resistor)
    Battery sense          -> GPIO 34  (optional, through a 100k/100k voltage divider)

  REQUIRED LIBRARIES (Arduino Library Manager / PlatformIO)
    - I2Cdevlib-MPU6050 + I2Cdevlib-Core
      NOTE: must be a fork that includes CalibrateAccel()/CalibrateGyro(),
      e.g. the "ElectronicCats/MPU6050" fork -- the original jrowberg
      i2cdevlib base class does not include these calibration helpers.
    - ESP32-BLE-Gamepad (lemmingDev)
    - Adafruit SSD1306 + Adafruit GFX

  SETUP NOTES
    - Run the MPU6050_6Axis_MotionApps20 "DMP6" example once on your
      specific chip to get its unique gyro/accel offsets, then paste
      them into the setX/Y/ZGyroOffset() / setZAccelOffset() calls in
      setup() below (currently left at 0 as placeholders).
    - All tuning constants (deadzone, sensitivity curve, smoothing,
      stillness gate) are grouped near the top of this file and are
      safe to tweak without touching any logic further down.

  LICENSE: MIT — do whatever you like with this, attribution appreciated.
  ============================================================
*/

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <BleGamepad.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- OBJECTS ----------
MPU6050 mpu;
BleGamepad bleGamepad("Gesture Glove", "DIY-Labs", 100);
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// ---------- PIN MAP ----------
#define PIN_BTN_INDEX  27  // dual-purpose: tap = jump, hold = mode switch
#define PIN_BTN_MIDDLE 26  // shoot
#define PIN_BTN_RING   32  // reload
#define PIN_BTN_PINKY  33  // melee
#define PIN_VIBE       25
#define PIN_BATT       34

// ---------- LONG-PRESS TUNING (two tiers on the same button) ----------
const unsigned long MODE_SWITCH_MS   = 600;  // hold past this = mode switch
const unsigned long RECENTER_HOLD_MS = 1800; // hold past this = re-center neutral pose

// ---------- POSE CALIBRATION (fixes drift + "lost neutral position") ----------
float pitchOffset = 0;
float rollOffset  = 0;
float filtPitch   = 0;   // smoothed pitch, reduces jitter/sensitivity
float filtRoll    = 0;   // smoothed roll
const float SMOOTHING_ALPHA = 0.10; // lower = smoother but slower to respond, higher = snappier but more jittery (was 0.15)
bool poseCalibrated = false;

// ---------- DMP STATE ----------
bool dmpReady = false;
uint16_t packetSize = 0;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3]; // yaw, pitch, roll (radians)
String dmpErrorMsg = "";

// ---------- GAME MODES ----------
// MODE_MENU is reserved for a future third mode (e.g. a settings/tuning
// screen navigated by tilt) -- the index-button mode switch currently
// only cycles RACING <-> FPS (see handleButtons()), by design.
enum GameMode { MODE_RACING, MODE_FPS, MODE_MENU };
GameMode currentMode = MODE_RACING;
const char* modeNames[] = {"RACING", "FPS / AIM", "MENU"};

// ---------- TUNING ----------
const float DEADZONE_DEG   = 8.0;   // ignore small hand tremor (was 6.0 -- widened further)
const float MAX_TILT_DEG   = 65.0;  // tilt angle for full axis travel (was 55.0 -- needs a big, deliberate tilt now)
const float EXPO_CURVE     = 2.4;   // steeper curve = even gentler near center (was 1.8)
const float YAW_FLICK_THRESH = 25.0; // deg/sec-ish flick to trigger a "boost" button
const unsigned long BUTTON_DEBOUNCE_MS = 20; // mechanical switch settle time

// ---------- SLEW RATE LIMIT (prevents sudden steering snaps) ----------
const int MAX_AXIS_STEP = 1400; // max change allowed per report cycle (was 2200 -- slower ramp now)
int lastRollAxis = 0;
int lastPitchAxis = 0;

// ---------- STILLNESS GATE (fixes residual drift/twitch while truly still) ----------
// A single-frame "how much did it change since last sample" check turned out
// too sensitive to normal DMP jitter (it never truly settled into "still").
// This instead looks at a WINDOW of the last N samples and checks the total
// spread (max-min) across all of them. Real stillness = a small, tight window.
// Real movement = the window spreads out fast. Much more robust.
const int STILL_WINDOW = 15;
float rollWindow[STILL_WINDOW];
float pitchWindow[STILL_WINDOW];
int windowIndex = 0;
bool windowFilled = false;
const float STILLNESS_RANGE_DEG = 3.5; // if the last 15 samples span less than this, force-lock to 0

// ---------- STATE TRACKING ----------
unsigned long lastModePress = 0;
unsigned long lastYawFlick = 0;
float yawBaseline = 0;
bool bleWasConnected = false;

// index-button long-press tracking
unsigned long indexPressStart = 0;
bool indexWasDown = false;
bool indexLongPressFired = false;   // fires once for mode switch
bool indexRecenterFired = false;    // fires once for re-center (deeper hold)

// ---------- BUTTON DEBOUNCE ----------
// Filters out mechanical switch bounce so a single physical press can't
// get misread as several rapid presses.
struct DebouncedButton {
  bool lastRaw = HIGH;   // last raw electrical reading
  bool stable = HIGH;    // the filtered, trustworthy state
  unsigned long lastChangeTime = 0;
};
DebouncedButton dbIndex, dbMiddle, dbRing, dbPinky;

bool readDebounced(int pin, DebouncedButton &btn) {
  bool raw = digitalRead(pin);
  if (raw != btn.lastRaw) {
    btn.lastChangeTime = millis();
    btn.lastRaw = raw;
  }
  if (millis() - btn.lastChangeTime > BUTTON_DEBOUNCE_MS) {
    btn.stable = raw;
  }
  return btn.stable; // LOW = pressed (INPUT_PULLUP wiring)
}

// =====================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000);

  pinMode(PIN_BTN_INDEX, INPUT_PULLUP);
  pinMode(PIN_BTN_MIDDLE, INPUT_PULLUP);
  pinMode(PIN_BTN_RING, INPUT_PULLUP);
  pinMode(PIN_BTN_PINKY, INPUT_PULLUP);
  pinMode(PIN_VIBE, OUTPUT);

  // ---- OLED init ----
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("GESTURE GLOVE");
  display.println("Booting...");
  display.display();

  // ---- MPU6050 + DMP init ----
  mpu.initialize();
  uint8_t dmpStatus = mpu.dmpInitialize();

  // These offsets are unique to YOUR chip. Run the DMP6 example's
  // calibration sketch once, note the offsets, and paste them here.
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(0);

  if (dmpStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize(); // ask the chip, don't hardcode 42
    dmpReady = true;
    Serial.print("DMP ready. Packet size = ");
    Serial.println(packetSize);
  } else {
    dmpErrorMsg = "DMP init code: " + String(dmpStatus);
    Serial.println("MPU6050 DMP FAILED -> " + dmpErrorMsg);
    Serial.println("Check: wiring (SDA=21,SCL=22), power (3.3V), I2C address (0x68).");
  }

  // ---- BLE Gamepad config ----
  BleGamepadConfiguration cfg;
  cfg.setAutoReport(false);
  cfg.setControllerType(CONTROLLER_TYPE_GAMEPAD);
  cfg.setButtonCount(5); // jump, shoot, reload, melee, yaw-flick boost
  cfg.setWhichAxes(true, true, true, true, false, false, false, false); // X, Y, Z, rZ
  bleGamepad.begin(&cfg);

  vibePulse(1); // ready buzz

  // ---- Auto-calibrate neutral pose ----
  // Whatever position the hand is resting in right now becomes "center."
  // This is what solves "forgetting the starting position" on every power-up.
  if (dmpReady) {
    calibrateNeutralPose();
  }
}

// =====================================================
// Holds the glove still for ~1.5s right after boot, reads the settled
// orientation, and stores it as the new zero-reference for pitch/roll.
// Also re-usable mid-session by a long button hold (see handleButtons()).
void calibrateNeutralPose() {
  display.clearDisplay();
  display.setCursor(0, 10);
  display.println("Calibrating...");
  display.println("Hold hand still");
  display.println("in neutral pose");
  display.display();

  unsigned long start = millis();
  while (millis() - start < 1500) {
    readOrientation(); // keep pumping the DMP FIFO so we get fresh data
    delay(15);
  }

  pitchOffset = ypr[1];
  rollOffset  = ypr[2];
  filtPitch = pitchOffset; // start the smoothing filter at the new center, no snap
  filtRoll  = rollOffset;
  poseCalibrated = true;

  vibePulse(2); // double buzz = "calibration done"
}

// =====================================================
void loop() {
  if (dmpReady) {
    readOrientation();
  }

  handleButtons();
  handleYawFlick();

  if (bleGamepad.isConnected()) {
    if (!bleWasConnected) { vibePulse(2); bleWasConnected = true; }
    sendGamepadFrame();
  } else {
    bleWasConnected = false;
  }

  updateDisplay();
  delay(15); // ~66Hz update rate, smooth but not flooding BLE
}

// =====================================================
// Reads DMP FIFO, converts quaternion -> yaw/pitch/roll in degrees
void readOrientation() {
  uint8_t mpuIntStatus = mpu.getIntStatus();
  uint16_t fifoCount = mpu.getFIFOCount();

  // ---- FIFO overflow recovery ----
  // If this ever happens, the buffer got backed up and stale/garbage data
  // would keep coming out otherwise. Resetting lets it lock on cleanly again.
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println("FIFO overflow -> reset");
    return;
  }

  if ((mpuIntStatus & 0x02) && fifoCount >= packetSize) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // convert radians -> degrees for human-readable logic
    ypr[0] *= 180 / M_PI;
    ypr[1] *= 180 / M_PI;
    ypr[2] *= 180 / M_PI;

    // ---- Smoothing filter ----
    // Blend the new reading into the running smoothed value instead of
    // using it raw. This is what kills the jittery/over-sensitive feel.
    filtPitch = filtPitch + SMOOTHING_ALPHA * (ypr[1] - filtPitch);
    filtRoll  = filtRoll  + SMOOTHING_ALPHA * (ypr[2] - filtRoll);
  }
}

// =====================================================
// Core logic: map tilt angles to analog stick values with deadzone + expo curve.
// EXPO_CURVE > 1.0 means: small tilts near center produce SMALL, gentle stick
// movement (fine control for slight steering corrections), while only tilting
// close to MAX_TILT_DEG unlocks the full range. This is how real force-feedback
// racing wheels feel -- not a straight 1:1 ramp, which feels twitchy near center.
int mapAxis(float angleDeg) {
  if (fabs(angleDeg) < DEADZONE_DEG) return 0;

  float sign = (angleDeg > 0) ? 1.0 : -1.0;
  float magnitude = fabs(angleDeg) - DEADZONE_DEG;
  float maxRange = MAX_TILT_DEG - DEADZONE_DEG;
  magnitude = constrain(magnitude, 0, maxRange);

  float normalized = magnitude / maxRange;         // 0.0 .. 1.0
  float curved = pow(normalized, EXPO_CURVE);       // bends the curve gentle-then-steep

  int value = (int)(curved * 32767.0);
  return (int)(sign * value);
}

// =====================================================
// Slew-rate limiter: caps how much the axis value is allowed to jump
// in a single report cycle. Smooths out any remaining sudden spikes
// so the in-game car/character never "snaps" instantly to full lock.
int applySlewLimit(int target, int &lastValue) {
  int delta = target - lastValue;
  if (delta > MAX_AXIS_STEP) delta = MAX_AXIS_STEP;
  if (delta < -MAX_AXIS_STEP) delta = -MAX_AXIS_STEP;
  lastValue += delta;
  return lastValue;
}

// =====================================================
// Pushes a new sample into the rolling window and returns true if the
// spread (max - min) across the whole window is small -- i.e. genuinely still.
bool updateStillness(float rollSample, float pitchSample) {
  rollWindow[windowIndex] = rollSample;
  pitchWindow[windowIndex] = pitchSample;
  windowIndex = (windowIndex + 1) % STILL_WINDOW;
  if (windowIndex == 0) windowFilled = true;

  int count = windowFilled ? STILL_WINDOW : windowIndex;
  if (count < STILL_WINDOW) return false; // not enough history yet -- assume moving

  float rollMin = rollWindow[0], rollMax = rollWindow[0];
  float pitchMin = pitchWindow[0], pitchMax = pitchWindow[0];
  for (int i = 1; i < STILL_WINDOW; i++) {
    if (rollWindow[i] < rollMin) rollMin = rollWindow[i];
    if (rollWindow[i] > rollMax) rollMax = rollWindow[i];
    if (pitchWindow[i] < pitchMin) pitchMin = pitchWindow[i];
    if (pitchWindow[i] > pitchMax) pitchMax = pitchWindow[i];
  }

  float rollRange = rollMax - rollMin;
  float pitchRange = pitchMax - pitchMin;
  return (rollRange < STILLNESS_RANGE_DEG) && (pitchRange < STILLNESS_RANGE_DEG);
}

void sendGamepadFrame() {
  // Subtract the calibrated neutral pose, then use the smoothed value —
  // this is what makes "flat hand" always mean "centered stick" no matter
  // what angle you happened to power on at.
  float centeredRoll  = filtRoll  - rollOffset;
  float centeredPitch = filtPitch - pitchOffset;

  // ---- Stillness gate (windowed) ----
  // If the last 15 samples all stayed within a tight range, treat it as
  // genuinely still and force output to exactly 0, regardless of any
  // small residual noise the deadzone alone might let slip through.
  bool isStill = updateStillness(centeredRoll, centeredPitch);

  int rollAxis  = isStill ? 0 : mapAxis(centeredRoll);   // roll  -> steering / strafe (X)
  int pitchAxis = isStill ? 0 : mapAxis(centeredPitch);  // pitch -> throttle / look-up-down (Y)

  // Smooth out any remaining sudden jumps before sending
  rollAxis  = applySlewLimit(rollAxis, lastRollAxis);
  pitchAxis = applySlewLimit(pitchAxis, lastPitchAxis);

  switch (currentMode) {
    case MODE_RACING:
      bleGamepad.setAxes(rollAxis, -pitchAxis, 0, 0);
      break;
    case MODE_FPS:
      // in FPS mode, roll/pitch become look X/Y (Z/rZ axes),
      // left stick (X/Y) stays centered for WASD-style movement via buttons
      bleGamepad.setAxes(0, 0, rollAxis, -pitchAxis);
      break;
    default:
      bleGamepad.setAxes(0, 0, 0, 0);
  }

  bleGamepad.sendReport();
}

// =====================================================
// Physical buttons: index finger is dual-purpose (tap=jump, hold=mode switch),
// the other three fingers are plain one-to-one button presses.
void handleButtons() {

  // ---- Middle / Ring / Pinky: simple direct mapping (debounced) ----
  if (readDebounced(PIN_BTN_MIDDLE, dbMiddle) == LOW) bleGamepad.press(BUTTON_2); else bleGamepad.release(BUTTON_2); // shoot
  if (readDebounced(PIN_BTN_RING,   dbRing)   == LOW) bleGamepad.press(BUTTON_3); else bleGamepad.release(BUTTON_3); // reload
  if (readDebounced(PIN_BTN_PINKY,  dbPinky)  == LOW) bleGamepad.press(BUTTON_4); else bleGamepad.release(BUTTON_4); // melee

  // ---- Index finger: tap vs hold vs deep-hold logic (debounced) ----
  bool indexIsDown = (readDebounced(PIN_BTN_INDEX, dbIndex) == LOW);

  if (indexIsDown && !indexWasDown) {
    // finger just pressed down -> start timing
    indexPressStart = millis();
    indexLongPressFired = false;
    indexRecenterFired = false;
  }

  unsigned long heldFor = millis() - indexPressStart;

  if (indexIsDown && !indexRecenterFired && heldFor > RECENTER_HOLD_MS) {
    // held really long -> re-center neutral pose (overrides mode switch for this hold)
    pitchOffset = filtPitch;
    rollOffset  = filtRoll;
    poseCalibrated = true;
    vibePulse(3); // triple buzz = "re-centered"
    indexRecenterFired = true;
    indexLongPressFired = true; // also suppress the mode-switch/jump actions for this hold
  }
  else if (indexIsDown && !indexLongPressFired && heldFor > MODE_SWITCH_MS) {
    // held past the shorter threshold -> mode switch
    currentMode = (GameMode)((currentMode + 1) % 2); // cycle RACING <-> FPS
    yawBaseline = ypr[0]; // re-zero yaw reference on mode switch
    vibePulse(1);
    indexLongPressFired = true;
  }

  if (!indexIsDown && indexWasDown && !indexLongPressFired) {
    // released quickly, before any hold threshold -> it was a tap -> jump
    bleGamepad.press(BUTTON_1);
    bleGamepad.sendReport();
    delay(40); // brief hold so the game registers the tap
    bleGamepad.release(BUTTON_1);
  }

  indexWasDown = indexIsDown;
}

// =====================================================
// Detects a fast yaw "flick" gesture (quick snap of the wrist)
// and fires it as a one-shot button press — e.g. jump / boost / reload
void handleYawFlick() {
  float yawDelta = ypr[0] - yawBaseline;
  if (fabs(yawDelta) > YAW_FLICK_THRESH && millis() - lastYawFlick > 500) {
    lastYawFlick = millis();
    bleGamepad.press(BUTTON_5); // boost/dash - triggered by a wrist flick, no finger needed
    delay(60); // short hold so the game registers the tap
    bleGamepad.release(BUTTON_5);
    vibePulse(1);
  }
  yawBaseline = yawBaseline * 0.98 + ypr[0] * 0.02; // slow drift-following baseline
}

// =====================================================
// Simple non-blocking-ish haptic buzz (fine to keep it a tiny blocking call, it's short)
void vibePulse(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PIN_VIBE, HIGH);
    delay(80);
    digitalWrite(PIN_VIBE, LOW);
    delay(80);
  }
}

// =====================================================
// Battery voltage sensing is OPTIONAL. If you're not wiring the
// divider, this just always returns -1 and the display shows "N/A".
int getBatteryPercent() {
  // Uncomment these two lines once you wire the voltage divider to PIN_BATT:
  // int raw = analogRead(PIN_BATT);
  // float voltage = (raw / 4095.0) * 3.3 * 2;
  // int pct = map(voltage * 100, 330, 420, 0, 100);
  // return constrain(pct, 0, 100);
  return -1; // no battery sensing wired
}

// =====================================================
void updateDisplay() {
  display.clearDisplay();

  if (!dmpReady) {
    // Persistent error screen - won't get overwritten away like before
    display.setCursor(0, 0);
    display.println("MPU6050 INIT FAILED");
    display.println(dmpErrorMsg);
    display.println("Check wiring & power");
    display.display();
    return;
  }

  display.setCursor(0, 0);
  display.print("Mode: "); display.println(modeNames[currentMode]);

  display.setCursor(0, 12);
  display.print("BLE: ");
  display.println(bleGamepad.isConnected() ? "Connected" : "Waiting...");

  display.setCursor(0, 24);
  int battPct = getBatteryPercent();
  display.print("Batt: ");
  if (battPct < 0) display.println("N/A");
  else { display.print(battPct); display.println("%"); }

  display.setCursor(0, 40);
  display.print("P:"); display.print((int)(filtPitch - pitchOffset));
  display.print(" R:"); display.print((int)(filtRoll - rollOffset));
  display.print(" Y:"); display.println((int)ypr[0]);

  display.display();
}
