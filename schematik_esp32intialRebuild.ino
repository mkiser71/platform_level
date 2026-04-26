// ESP32 Trailer Leveling Controller
// Controls 4 scissor jacks via IBT2 motor drivers using MPU6050 accelerometer
// Displays status on SSD1306 OLED, with manual/auto modes and safety e-stop

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================
// OLED I2C
#define OLED_SDA 21
#define OLED_SCL 22

// MPU6050 I2C (shared bus)
#define MPU_SDA 21
#define MPU_SCL 22
#define MPU_INT 4
#define MPU_AD0 5

// Motor 0 (Front Left) - dc-motor_2
#define M0_IN1 2
#define M0_IN2 18
#define M0_EN  19

// Motor 1 (Front Right) - dc-motor_3
#define M1_IN1 13
#define M1_IN2 14
#define M1_EN  16

// Motor 2 (Rear Left) - dc-motor_4
#define M2_IN1 17
#define M2_IN2 23
#define M2_EN  25

// Motor 3 (Rear Right) - dc-motor_5
#define M3_IN1 26
#define M3_IN2 27
#define M3_EN  32

// Buttons (GPIO34 is input-only, suitable)
#define BTN_SELECT   33  // Select / confirm button
#define BTN_NEXT     34  // Next / cycle button (input-only GPIO, fine)

// PWM channels
#define PWM_CH0 0
#define PWM_CH1 1
#define PWM_CH2 2
#define PWM_CH3 3
#define PWM_FREQ 5000
#define PWM_RES  8

// ============================================================
// DISPLAY SETUP
// ============================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MPU6050 mpu;

// ============================================================
// SYSTEM STATE
// ============================================================
enum SystemMode {
  MODE_SENSOR_READOUT,  // Show pitch/roll only
  MODE_MANUAL,          // Manual motor control via buttons (menu driven)
  MODE_AUTO,            // Automatic leveling
  MODE_MENU,            // Menu navigation
  MODE_CALIBRATION,     // Calibration routine
  MODE_ESTOP            // Emergency stop
};

enum MenuState {
  MENU_MAIN,
  MENU_MODE_SELECT,
  MENU_ROLL_LIMIT,
  MENU_CALIBRATE,
  MENU_MANUAL_JACK
};

SystemMode currentMode = MODE_SENSOR_READOUT;
MenuState  currentMenu = MENU_MAIN;

// ============================================================
// LEVELING CONFIGURATION
// ============================================================
float pitchOffset = 0.0f;
float rollOffset  = 0.0f;

bool  rollLimitEnabled  = false;   // Limit roll correction
float levelThreshold    = 0.5f;    // Degrees considered "level"
float maxAutoSpeed      = 180;     // PWM 0-255 for auto mode
float manualSpeed       = 200;     // PWM for manual mode

// Current measured pitch/roll
float currentPitch = 0.0f;
float currentRoll  = 0.0f;

// Calibration averaging
#define CAL_SAMPLES 100

// ============================================================
// JACK / MOTOR STATE
// ============================================================
// Motor indices: 0=FrontLeft, 1=FrontRight, 2=RearLeft, 3=RearRight
struct Motor {
  uint8_t in1;
  uint8_t in2;
  uint8_t en;
  uint8_t pwmCh;
};

Motor motors[4] = {
  {M0_IN1, M0_IN2, M0_EN, PWM_CH0},
  {M1_IN1, M1_IN2, M1_EN, PWM_CH1},
  {M2_IN1, M2_IN2, M2_EN, PWM_CH2},
  {M3_IN1, M3_IN2, M3_EN, PWM_CH3}
};

const char* motorNames[4] = {"FL Jack", "FR Jack", "RL Jack", "RR Jack"};

// ============================================================
// BUTTON DEBOUNCE
// ============================================================
#define DEBOUNCE_MS 50
#define LONG_PRESS_MS 1000

struct ButtonState {
  bool lastRaw;
  bool pressed;        // edge detected press
  bool longPressed;
  unsigned long pressTime;
};

ButtonState btnSelect = {true, false, false, 0};
ButtonState btnNext   = {true, false, false, 0};

// ============================================================
// MENU DATA
// ============================================================
// Main menu items
const char* mainMenuItems[] = {
  "Sensor Readout",
  "Manual Mode",
  "Auto Level",
  "Roll Limit",
  "Calibrate",
  "E-STOP"
};
const int mainMenuCount = 6;
int menuCursor = 0;

// Manual jack sub-menu
int  selectedJack = 0;   // which jack is being manually controlled

// ============================================================
// TIMING
// ============================================================
unsigned long lastSensorRead   = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastAutoUpdate   = 0;
#define SENSOR_INTERVAL   50
#define DISPLAY_INTERVAL  200
#define AUTO_INTERVAL     100

// ============================================================
// FUNCTION PROTOTYPES (implemented fully below)
// ============================================================

// Motor control
void motorStop(int idx);
void motorExtend(int idx, uint8_t speed);
void motorRetract(int idx, uint8_t speed);
void stopAllMotors();

// Sensor
void readIMU();

// Display helpers
void displayClear();
void displayHeader(const char* title);
void showSensorReadout();
void showMenu();
void showManualMode();
void showAutoStatus();
void showCalibration();
void showEstop();
void showRollLimitMenu();
void showManualJackMenu();

// Button handling
void updateButtons();
bool btnSelectPressed();
bool btnSelectLong();
bool btnNextPressed();

// Auto leveling
void runAutoLevel();

// Calibration
void runCalibration();

// ============================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================
void motorStop(int idx) {
  digitalWrite(motors[idx].in1, LOW);
  digitalWrite(motors[idx].in2, LOW);
  ledcWrite(motors[idx].pwmCh, 0);
}

void motorExtend(int idx, uint8_t speed) {
  // IBT2: IN1=HIGH, IN2=LOW = forward
  digitalWrite(motors[idx].in1, HIGH);
  digitalWrite(motors[idx].in2, LOW);
  ledcWrite(motors[idx].pwmCh, speed);
}

void motorRetract(int idx, uint8_t speed) {
  // IBT2: IN1=LOW, IN2=HIGH = reverse
  digitalWrite(motors[idx].in1, LOW);
  digitalWrite(motors[idx].in2, HIGH);
  ledcWrite(motors[idx].pwmCh, speed);
}

void stopAllMotors() {
  for (int i = 0; i < 4; i++) {
    motorStop(i);
  }
}

// ============================================================
// SENSOR READ
// ============================================================
void readIMU() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate pitch and roll from accelerometer
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  // Pitch: rotation around Y axis (front-back tilt)
  float pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
  // Roll: rotation around X axis (side-to-side tilt)
  float roll  = atan2(ay, sqrt(ax * ax + az * az)) * 180.0f / PI;

  currentPitch = pitch - pitchOffset;
  currentRoll  = roll  - rollOffset;
}

// ============================================================
// BUTTON UPDATE
// ============================================================
void updateButtons() {
  btnSelect.pressed     = false;
  btnSelect.longPressed = false;
  btnNext.pressed       = false;
  btnNext.longPressed   = false;

  // SELECT button - GPIO33 active LOW (pull-up)
  bool rawSel = (digitalRead(BTN_SELECT) == LOW);
  if (rawSel && !btnSelect.lastRaw) {
    btnSelect.pressTime = millis();
  }
  if (!rawSel && btnSelect.lastRaw) {
    unsigned long held = millis() - btnSelect.pressTime;
    if (held >= DEBOUNCE_MS && held < LONG_PRESS_MS) {
      btnSelect.pressed = true;
    } else if (held >= LONG_PRESS_MS) {
      btnSelect.longPressed = true;
    }
  }
  btnSelect.lastRaw = rawSel;

  // NEXT button - GPIO34 active LOW (pull-up not available on input-only, use external pull-up or check polarity)
  // GPIO34 has no internal pull-up, external pull-up assumed to 3.3V
  bool rawNxt = (digitalRead(BTN_NEXT) == LOW);
  if (rawNxt && !btnNext.lastRaw) {
    btnNext.pressTime = millis();
  }
  if (!rawNxt && btnNext.lastRaw) {
    unsigned long held = millis() - btnNext.pressTime;
    if (held >= DEBOUNCE_MS && held < LONG_PRESS_MS) {
      btnNext.pressed = true;
    } else if (held >= LONG_PRESS_MS) {
      btnNext.longPressed = true;
    }
  }
  btnNext.lastRaw = rawNxt;
}

bool btnSelectPressed() { return btnSelect.pressed; }
bool btnSelectLong()    { return btnSelect.longPressed; }
bool btnNextPressed()   { return btnNext.pressed; }

// ============================================================
// DISPLAY FUNCTIONS
// ============================================================
void displayClear() {
  display.clearDisplay();
}

void displayHeader(const char* title) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(title);
  // Draw separator line
  display.drawFastHLine(0, 9, SCREEN_WIDTH, SSD1306_WHITE);
}

void showSensorReadout() {
  displayClear();
  displayHeader("Pitch & Roll Monitor");
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 14);
  display.print("Pitch: ");
  display.print(currentPitch, 2);
  display.print((char)247); // degree symbol
  if (currentPitch > levelThreshold)       display.print(" FWD");
  else if (currentPitch < -levelThreshold) display.print(" BWD");
  else                                      display.print(" LVL");

  display.setCursor(0, 26);
  display.print("Roll:  ");
  display.print(currentRoll, 2);
  display.print((char)247);
  if (currentRoll > levelThreshold)       display.print(" R");
  else if (currentRoll < -levelThreshold) display.print(" L");
  else                                    display.print(" LVL");

  display.setCursor(0, 40);
  display.print("Roll Lim: ");
  display.print(rollLimitEnabled ? "ON " : "OFF");

  display.setCursor(0, 52);
  display.setTextSize(1);
  display.print("[SEL]=Menu [NXT]=Cycle");
  display.display();
}

void showMenu() {
  displayClear();
  displayHeader("  === MENU ===");

  int startItem = (menuCursor > 3) ? menuCursor - 3 : 0;
  int lineY = 13;
  for (int i = startItem; i < mainMenuCount && lineY < 55; i++) {
    display.setCursor(0, lineY);
    if (i == menuCursor) {
      display.print("> ");
    } else {
      display.print("  ");
    }
    display.print(mainMenuItems[i]);
    lineY += 10;
  }

  display.setCursor(0, 56);
  display.print("[SEL]=OK [NXT]=Scroll");
  display.display();
}

void showManualMode() {
  displayClear();
  displayHeader("Manual Jack Control");

  display.setTextSize(1);
  display.setCursor(0, 13);
  display.print("Jack: ");
  display.print(motorNames[selectedJack]);

  display.setCursor(0, 26);
  display.print("[NXT]=Change Jack");
  display.setCursor(0, 36);
  display.print("[SEL]=Extend");
  display.setCursor(0, 46);
  display.print("[SEL Long]=Retract");
  display.setCursor(0, 56);
  display.print("[NXT Long]=Exit");
  display.display();
}

void showAutoStatus() {
  displayClear();
  displayHeader("Auto Leveling");

  display.setTextSize(1);
  display.setCursor(0, 13);
  display.print("Pitch: ");
  display.print(currentPitch, 2);
  display.print((char)247);

  display.setCursor(0, 23);
  display.print("Roll:  ");
  display.print(currentRoll, 2);
  display.print((char)247);

  display.setCursor(0, 35);
  bool pitchOk = (abs(currentPitch) <= levelThreshold);
  bool rollOk  = rollLimitEnabled ? true : (abs(currentRoll) <= levelThreshold);
  if (pitchOk && rollOk) {
    display.print("STATUS: LEVEL!");
  } else {
    display.print("STATUS: Adjusting...");
  }

  display.setCursor(0, 47);
  display.print("Roll Limit: ");
  display.print(rollLimitEnabled ? "ON" : "OFF");

  display.setCursor(0, 56);
  display.print("[SEL]=Stop [NXT]=Menu");
  display.display();
}

void showEstop() {
  displayClear();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 10);
  display.println("!! ESTOP !!");
  display.setTextSize(1);
  display.setCursor(0, 40);
  display.println("All motors stopped.");
  display.setCursor(0, 52);
  display.println("[SEL]=Resume Menu");
  display.display();
}

void showRollLimitMenu() {
  displayClear();
  displayHeader("Roll Limit Setting");

  display.setTextSize(1);
  display.setCursor(0, 14);
  display.print("Roll Limit Correction");
  display.setCursor(0, 26);
  display.print("Currently: ");
  display.print(rollLimitEnabled ? "ENABLED" : "DISABLED");
  display.setCursor(0, 40);
  display.print("[NXT]=Toggle");
  display.setCursor(0, 52);
  display.print("[SEL]=Save & Back");
  display.display();
}

void showCalibration() {
  displayClear();
  displayHeader("Calibration");
  display.setTextSize(1);
  display.setCursor(0, 14);
  display.println("Place trailer on");
  display.println("a known level surface.");
  display.println("");
  display.println("[SEL]=Start Cal");
  display.println("[NXT]=Cancel");
  display.display();
}

void showManualJackMenu() {
  showManualMode();
}

// ============================================================
// AUTO LEVELING LOGIC
// ============================================================
// Strategy:
// Pitch correction: adjust front pair vs rear pair
// Roll correction:  adjust left pair vs right pair (if roll limit not engaged)
//
// Jack layout (looking from above):
//   Motor 0 = Front Left (FL)
//   Motor 1 = Front Right (FR)
//   Motor 2 = Rear Left  (RL)
//   Motor 3 = Rear Right (RR)
//
// Positive pitch = front low  -> extend FL, FR; retract RL, RR
// Negative pitch = rear low   -> extend RL, RR; retract FL, FR
// Positive roll  = right low  -> extend FR, RR; retract FL, RL
// Negative roll  = left low   -> extend FL, RL; retract FR, RR

void runAutoLevel() {
  bool pitchOk = (abs(currentPitch) <= levelThreshold);
  bool rollOk  = rollLimitEnabled ? true : (abs(currentRoll) <= levelThreshold);

  if (pitchOk && rollOk) {
    stopAllMotors();
    return;
  }

  // Determine per-motor direction: +1 extend, -1 retract, 0 stop
  // We accumulate contributions from pitch and roll corrections
  float motorCmd[4] = {0, 0, 0, 0}; // FL, FR, RL, RR

  // --- Pitch correction ---
  if (!pitchOk) {
    float pErr = currentPitch;
    if (pErr > levelThreshold) {
      // Front too high relative to rear OR rear too low - extend rear, retract front
      // Actually: positive pitch means front tilted down (front low)
      // atan2(ax,...): if ax positive, front is low
      // Extend front jacks, retract rear
      motorCmd[0] += 1;  // FL extend
      motorCmd[1] += 1;  // FR extend
      motorCmd[2] -= 1;  // RL retract
      motorCmd[3] -= 1;  // RR retract
    } else if (pErr < -levelThreshold) {
      motorCmd[0] -= 1;  // FL retract
      motorCmd[1] -= 1;  // FR retract
      motorCmd[2] += 1;  // RL extend
      motorCmd[3] += 1;  // RR extend
    }
  }

  // --- Roll correction (only if not limited) ---
  if (!rollLimitEnabled && !rollOk) {
    float rErr = currentRoll;
    if (rErr > levelThreshold) {
      // Right side low - extend right, retract left
      motorCmd[0] -= 1;  // FL retract
      motorCmd[1] += 1;  // FR extend
      motorCmd[2] -= 1;  // RL retract
      motorCmd[3] += 1;  // RR extend
    } else if (rErr < -levelThreshold) {
      motorCmd[0] += 1;  // FL extend
      motorCmd[1] -= 1;  // FR retract
      motorCmd[2] += 1;  // RL extend
      motorCmd[3] -= 1;  // RR retract
    }
  }

  // Apply commands
  for (int i = 0; i < 4; i++) {
    if (motorCmd[i] > 0) {
      motorExtend(i, (uint8_t)maxAutoSpeed);
    } else if (motorCmd[i] < 0) {
      motorRetract(i, (uint8_t)maxAutoSpeed);
    } else {
      motorStop(i);
    }
  }
}

// ============================================================
// CALIBRATION ROUTINE
// ============================================================
void runCalibration() {
  stopAllMotors();

  // Show "calibrating" message
  displayClear();
  displayHeader("Calibrating...");
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.println("Please wait...");
  display.println("Do not move trailer.");
  display.display();

  float sumPitch = 0.0f;
  float sumRoll  = 0.0f;

  for (int s = 0; s < CAL_SAMPLES; s++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;

    sumPitch += atan2(ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
    sumRoll  += atan2(ay, sqrt(ax * ax + az * az)) * 180.0f / PI;

    delay(20);

    // Progress bar
    if (s % 10 == 0) {
      displayClear();
      displayHeader("Calibrating...");
      display.setCursor(0, 20);
      display.print("Progress: ");
      display.print((s * 100) / CAL_SAMPLES);
      display.println("%");
      int barWidth = (s * 100) / CAL_SAMPLES;
      display.drawRect(0, 35, 128, 10, SSD1306_WHITE);
      display.fillRect(1, 36, barWidth, 8, SSD1306_WHITE);
      display.display();
    }
  }

  pitchOffset = sumPitch / CAL_SAMPLES;
  rollOffset  = sumRoll  / CAL_SAMPLES;

  // Show result
  displayClear();
  displayHeader("Calibration Done");
  display.setTextSize(1);
  display.setCursor(0, 14);
  display.print("P Offset: ");
  display.print(pitchOffset, 2);
  display.print((char)247);
  display.setCursor(0, 26);
  display.print("R Offset: ");
  display.print(rollOffset, 2);
  display.print((char)247);
  display.setCursor(0, 44);
  display.println("Offsets saved.");
  display.setCursor(0, 54);
  display.println("[SEL]=Return to Menu");
  display.display();

  // Wait for select
  while (true) {
    updateButtons();
    if (btnSelectPressed()) break;
    delay(50);
  }

  currentMode = MODE_MENU;
  currentMenu = MENU_MAIN;
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Trailer Leveling Controller Starting...");

  // Set MPU AD0 low for default I2C address 0x68
  pinMode(MPU_AD0, OUTPUT);
  digitalWrite(MPU_AD0, LOW);

  // Motor pins
  for (int i = 0; i < 4; i++) {
    pinMode(motors[i].in1, OUTPUT);
    pinMode(motors[i].in2, OUTPUT);
    digitalWrite(motors[i].in1, LOW);
    digitalWrite(motors[i].in2, LOW);

    ledcSetup(motors[i].pwmCh, PWM_FREQ, PWM_RES);
    ledcAttachPin(motors[i].en, motors[i].pwmCh);
    ledcWrite(motors[i].pwmCh, 0);
  }

  // Button pins
  pinMode(BTN_SELECT, INPUT_PULLUP);
  // GPIO34 has no internal pull-up; external 10k pull-up to 3.3V required
  pinMode(BTN_NEXT, INPUT);

  // I2C
  Wire.begin(OLED_SDA, OLED_SCL);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 init failed");
    while (true) { delay(1000); }
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Leveling Controller");
  display.println("  Initializing...");
  display.display();
  delay(1500);

  // MPU6050
  if (!mpu.begin()) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("MPU6050 not found!");
    display.println("Check wiring.");
    display.display();
    Serial.println("MPU6050 init failed");
    while (true) { delay(1000); }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Default mode: sensor readout
  currentMode = MODE_SENSOR_READOUT;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Ready!");
  display.display();
  delay(800);

  Serial.println("Setup complete.");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  unsigned long now = millis();

  // Read buttons every loop
  updateButtons();

  // Read IMU at sensor interval
  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;
    readIMU();
  }

  // ---- STATE MACHINE ----

  // E-STOP check: long press SELECT from anywhere exits estop, or entering it
  // Entering ESTOP handled in menu; once in ESTOP, SELECT exits to menu
  if (currentMode == MODE_ESTOP) {
    stopAllMotors();
    if (now - lastDisplayUpdate >= DISPLAY_INTERVAL) {
      lastDisplayUpdate = now;
      showEstop();
    }
    if (btnSelectPressed()) {
      currentMode = MODE_MENU;
      currentMenu = MENU_MAIN;
      menuCursor  = 0;
    }
    return;
  }

  // ----- SENSOR READOUT MODE -----
  if (currentMode == MODE_SENSOR_READOUT) {
    if (now - lastDisplayUpdate >= DISPLAY_INTERVAL) {
      lastDisplayUpdate = now;
      showSensorReadout();
    }
    if (btnSelectPressed()) {
      // Enter menu
      currentMode = MODE_MENU;
      currentMenu = MENU_MAIN;
      menuCursor  = 0;
    }
    if (btnNextPressed()) {
      // Cycle through info (future expansion placeholder)
    }
    return;
  }

  // ----- MENU MODE -----
  if (currentMode == MODE_MENU) {
    if (now - lastDisplayUpdate >= DISPLAY_INTERVAL) {
      lastDisplayUpdate = now;
      if (currentMenu == MENU_MAIN) {
        showMenu();
      } else if (currentMenu == MENU_ROLL_LIMIT) {
        showRollLimitMenu();
      } else if (currentMenu == MENU_CALIBRATE) {
        showCalibration();
      } else if (currentMenu == MENU_MANUAL_JACK) {
        showManualJackMenu();
      }
    }

    if (currentMenu == MENU_MAIN) {
      if (btnNextPressed()) {
        menuCursor = (menuCursor + 1) % mainMenuCount;
      }
      if (btnSelectPressed()) {
        switch (menuCursor) {
          case 0:  // Sensor Readout
            currentMode = MODE_SENSOR_READOUT;
            break;
          case 1:  // Manual Mode
            currentMode  = MODE_MANUAL;
            currentMenu  = MENU_MANUAL_JACK;
            selectedJack = 0;
            break;
          case 2:  // Auto Level
            currentMode = MODE_AUTO;
            break;
          case 3:  // Roll Limit
            currentMenu = MENU_ROLL_LIMIT;
            break;
          case 4:  // Calibrate
            currentMenu = MENU_CALIBRATE;
            break;
          case 5:  // E-STOP
            stopAllMotors();
            currentMode = MODE_ESTOP;
            break;
          default:
            break;
        }
      }
      // Long press NEXT from anywhere in menu = back to sensor readout
      if (btnNextPressed() && btnNext.longPressed) {
        currentMode = MODE_SENSOR_READOUT;
      }
    }

    else if (currentMenu == MENU_ROLL_LIMIT) {
      if (btnNextPressed()) {
        rollLimitEnabled = !rollLimitEnabled;
      }
      if (btnSelectPressed()) {
        // Save & return
        currentMenu = MENU_MAIN;
      }
    }

    else if (currentMenu == MENU_CALIBRATE) {
      if (btnSelectPressed()) {
        runCalibration();
        // runCalibration returns to menu internally
      }
      if (btnNextPressed()) {
        currentMenu = MENU_MAIN;
      }
    }

    else if (currentMenu == MENU_MANUAL_JACK) {
      // This sub-menu used only as display; control handled in MANUAL mode
      // Redirect to manual mode
      currentMode = MODE_MANUAL;
    }

    return;
  }

  // ----- MANUAL MODE -----
  if (currentMode == MODE_MANUAL) {
    if (now - lastDisplayUpdate >= DISPLAY_INTERVAL) {
      lastDisplayUpdate = now;
      showManualMode();
    }

    // NEXT short = change selected jack
    if (btnNextPressed()) {
      stopAllMotors();
      selectedJack = (selectedJack + 1) % 4;
    }

    // NEXT long = exit manual mode
    if (btnNext.longPressed) {
      stopAllMotors();
      currentMode = MODE_MENU;
      currentMenu = MENU_MAIN;
      return;
    }

    // SELECT short = extend selected jack (while held conceptually; we pulse)
    // Since we only have edge events, we drive while button is physically held
    bool selHeld = (digitalRead(BTN_SELECT) == LOW);
    bool nxtHeld = (digitalRead(BTN_NEXT)   == LOW);

    // Use raw held state for motor drive in manual
    // Extend while SELECT is physically held and NEXT is not
    if (selHeld && !nxtHeld) {
      motorExtend(selectedJack, (uint8_t)manualSpeed);
    }
    // Retract while NEXT is physically held and SELECT is not
    else if (nxtHeld && !selHeld) {
      motorRetract(selectedJack, (uint8_t)manualSpeed);
    }
    // Both held or neither: stop
    else {
      motorStop(selectedJack);
    }

    // Long SELECT = go to menu (detected on release by btnSelectLong)
    if (btnSelectLong()) {
      stopAllMotors();
      currentMode = MODE_MENU;
      currentMenu = MENU_MAIN;
    }

    return;
  }

  // ----- AUTO LEVEL MODE -----
  if (currentMode == MODE_AUTO) {
    if (now - lastDisplayUpdate >= DISPLAY_INTERVAL) {
      lastDisplayUpdate = now;
      showAutoStatus();
    }

    if (now - lastAutoUpdate >= AUTO_INTERVAL) {
      lastAutoUpdate = now;
      runAutoLevel();
    }

    // SELECT = stop and return to menu
    if (btnSelectPressed()) {
      stopAllMotors();
      currentMode = MODE_MENU;
      currentMenu = MENU_MAIN;
    }

    // NEXT = stop motors
    if (btnNextPressed()) {
      stopAllMotors();
    }

    // Long press NEXT = estop
    if (btnNext.longPressed) {
      stopAllMotors();
      currentMode = MODE_ESTOP;
    }

    return;
  }
}