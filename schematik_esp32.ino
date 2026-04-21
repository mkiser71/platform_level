// RV/Trailer Level Controller with 4 DC Motors (scissor jacks)
// Uses L298N-style motor drivers (or IBT-2), MPU6050 accelerometer, and manual override buttons
// Platform: ESP32
// Power: 12V, 30A DC from camper battery
//
// === IBT-2 vs L298N for this use-case ===
// IBT-2 (BTS7960) is an EXCELLENT and commercially viable choice for this application:
//   - Rated for 43A continuous, 43A peak per channel (well above your 30A supply)
//   - Low on-resistance (~10mΩ) = very little heat generation vs L298N's 2-4V drop
//   - Built-in current sensing and thermal protection
//   - Highly efficient H-bridge (MOSFET-based, ~95% efficiency vs ~70% for L298N)
//   - Used commercially in electric bikes, industrial automation, robotics
//   - Cost-effective ($3-8 per module at quantity)
//   - Each IBT-2 drives ONE motor bidirectionally using two PWM pins (RPWM/LPWM)
//     OR can be wired IN1/IN2/EN style with a PWM enable line
//   - For 12V/30A scissor jack motors, IBT-2 is the professional-grade recommendation
//
// L298N is NOT recommended here:
//   - Max 2A per channel (4A peak) - far too weak for scissor jack motors
//   - High voltage drop wastes battery power as heat
//   - Would require multiple modules and heatsinking
//
// This code uses the IN1/IN2/EN wiring scheme compatible with both IBT-2 and L298N-style drivers
// For IBT-2: Connect IN1->RPWM, IN2->LPWM, EN->R_EN and L_EN tied together
// The EN pin controls speed via PWM, IN1/IN2 control direction

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>

// ============================================================
// PIN DEFINITIONS (matching component manifest)
// ============================================================

// Motor 0 - Front Left Jack (dc-motor_0)
#define DC_MOTOR_0_IN1  5
#define DC_MOTOR_0_IN2  18
#define DC_MOTOR_0_EN   19

// Motor 1 - Front Right Jack (dc-motor_1)
#define DC_MOTOR_1_IN1  13
#define DC_MOTOR_1_IN2  14
#define DC_MOTOR_1_EN   16

// Motor 2 - Rear Left Jack (dc-motor_2)
#define DC_MOTOR_2_IN1  17
#define DC_MOTOR_2_IN2  23
#define DC_MOTOR_2_EN   25

// Motor 3 - Rear Right Jack (dc-motor_3)
#define DC_MOTOR_3_IN1  26
#define DC_MOTOR_3_IN2  27
#define DC_MOTOR_3_EN   32

// MPU-6050 I2C pins
#define MPU_SDA         21
#define MPU_SCL         22
#define MPU_INT         2
#define MPU_AD0         4

// Manual override pushbuttons
// Button 9: Mode toggle (AUTO/MANUAL)
#define BTN_MODE        12
// Button 10: Emergency stop / reset
#define BTN_ESTOP       3

// ============================================================
// PWM CHANNEL ASSIGNMENTS (ESP32 ledc)
// ============================================================
#define PWM_CH_M0   0
#define PWM_CH_M1   1
#define PWM_CH_M2   2
#define PWM_CH_M3   3

#define PWM_FREQ    5000
#define PWM_RES     8        // 8-bit: 0-255

// ============================================================
// CONFIGURATION CONSTANTS
// ============================================================
#define NUM_MOTORS           4
#define NUM_JACKS            4   // All 4 motors are leveling jacks

#define MOTOR_SPEED_FULL     200  // PWM duty cycle 0-255
#define MOTOR_SPEED_SLOW     120  // Slow speed for fine adjustment

// Accelerometer leveling thresholds (degrees)
#define LEVEL_THRESHOLD      0.5f   // Consider level within +/- 0.5 degrees
#define TILT_MAX             15.0f  // Maximum tilt before emergency stop

// Button debounce time (ms)
#define DEBOUNCE_MS          50

// Auto-leveling update interval (ms)
#define AUTO_LEVEL_INTERVAL  200

// Accelerometer read interval (ms)
#define SENSOR_READ_INTERVAL 100

// Safety timeout: max continuous motor run time (ms)
#define MOTOR_TIMEOUT_MS     30000  // 30 seconds max continuous run

// Status print interval (ms)
#define STATUS_PRINT_INTERVAL 2000

// ============================================================
// ENUMERATIONS
// ============================================================
enum MotorState {
  MOTOR_STOP = 0,
  MOTOR_EXTEND,   // Jack extends downward, lifting that corner of trailer
  MOTOR_RETRACT   // Jack retracts upward, lowering that corner of trailer
};

enum ControlMode {
  MODE_MANUAL = 0,
  MODE_AUTO
};

// ============================================================
// MOTOR STRUCTURE
// ============================================================
struct Motor {
  uint8_t  in1Pin;
  uint8_t  in2Pin;
  uint8_t  enPin;
  uint8_t  pwmChannel;
  MotorState state;
  unsigned long startTime;  // When motor last started
  bool     enabled;
  const char* name;
};

// ============================================================
// BUTTON STRUCTURE
// ============================================================
struct Button {
  uint8_t  pin;
  bool     lastReading;
  bool     stableState;
  unsigned long lastDebounceTime;
  bool     pressedEvent;  // True for one cycle when button is freshly pressed
};

// ============================================================
// GLOBAL OBJECTS AND VARIABLES
// ============================================================

MPU6050 mpu;

// Motor definitions - matched to component manifest pin assignments
Motor motors[NUM_MOTORS] = {
  {DC_MOTOR_0_IN1, DC_MOTOR_0_IN2, DC_MOTOR_0_EN, PWM_CH_M0, MOTOR_STOP, 0, true, "FL-Jack"},
  {DC_MOTOR_1_IN1, DC_MOTOR_1_IN2, DC_MOTOR_1_EN, PWM_CH_M1, MOTOR_STOP, 0, true, "FR-Jack"},
  {DC_MOTOR_2_IN1, DC_MOTOR_2_IN2, DC_MOTOR_2_EN, PWM_CH_M2, MOTOR_STOP, 0, true, "RL-Jack"},
  {DC_MOTOR_3_IN1, DC_MOTOR_3_IN2, DC_MOTOR_3_EN, PWM_CH_M3, MOTOR_STOP, 0, true, "RR-Jack"}
};

// Button definitions
// BTN_MODE  (pin 12): Toggle AUTO/MANUAL mode
// BTN_ESTOP (pin  3): Emergency stop / clear emergency stop
Button buttons[2] = {
  {BTN_MODE,  HIGH, HIGH, 0, false},
  {BTN_ESTOP, HIGH, HIGH, 0, false}
};
#define NUM_BUTTONS 2
#define BTN_IDX_MODE  0
#define BTN_IDX_ESTOP 1

// System state
ControlMode controlMode  = MODE_MANUAL;
bool        mpuAvailable = false;
bool        emergencyStop = false;

// Accelerometer angles
float pitchAngle = 0.0f;  // Front-back tilt (degrees)
float rollAngle  = 0.0f;  // Left-right tilt (degrees)

// Timing
unsigned long lastAutoLevelTime = 0;
unsigned long lastSensorRead    = 0;
unsigned long lastStatusPrint   = 0;

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================
void     initMotors();
void     initButtons();
bool     initMPU();
void     setMotor(uint8_t motorIndex, MotorState state, uint8_t speed);
void     stopMotor(uint8_t motorIndex);
void     stopAllMotors();
void     readAccelerometer();
void     autoLevelRoutine();
void     updateButtons();
bool     isButtonPressed(uint8_t btnIndex);
void     checkMotorTimeouts();
void     checkTiltSafety();
void     printStatus();

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("==============================================");
  Serial.println("  RV/Trailer Level Controller - ESP32");
  Serial.println("  Motor Driver: IBT-2 (BTS7960) recommended");
  Serial.println("==============================================");

  // Initialize I2C for MPU6050
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(400000); // 400kHz fast mode

  // AD0 low = I2C address 0x68
  pinMode(MPU_AD0, OUTPUT);
  digitalWrite(MPU_AD0, LOW);

  // Initialize MPU6050
  mpuAvailable = initMPU();
  if (mpuAvailable) {
    Serial.println("[OK] MPU6050 accelerometer initialized.");
  } else {
    Serial.println("[WARN] MPU6050 not found. Manual mode only.");
    controlMode = MODE_MANUAL;
  }

  // Initialize motors and PWM
  initMotors();
  Serial.println("[OK] Motors initialized.");

  // Initialize buttons
  initButtons();
  Serial.println("[OK] Buttons initialized.");

  // Safety: ensure all motors are stopped on startup
  stopAllMotors();

  Serial.println("[OK] All motors stopped.");
  Serial.println("[OK] System ready. Mode: MANUAL");
  Serial.println("----------------------------------------------");
  Serial.println("  BTN_MODE  (pin 12): Toggle AUTO/MANUAL");
  Serial.println("  BTN_ESTOP (pin  3): Emergency stop/reset");
  Serial.println("==============================================");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  unsigned long now = millis();

  // --- Update button debounce states ---
  updateButtons();

  // --- Handle mode toggle button ---
  if (isButtonPressed(BTN_IDX_MODE)) {
    if (controlMode == MODE_MANUAL) {
      if (mpuAvailable) {
        controlMode = MODE_AUTO;
        Serial.println("[MODE] Switched to AUTO leveling.");
      } else {
        Serial.println("[WARN] Cannot switch to AUTO: MPU6050 unavailable.");
      }
    } else {
      controlMode = MODE_MANUAL;
      stopAllMotors();
      Serial.println("[MODE] Switched to MANUAL. All motors stopped.");
    }
  }

  // --- Handle emergency stop button ---
  if (isButtonPressed(BTN_IDX_ESTOP)) {
    if (emergencyStop) {
      // Clear emergency stop
      emergencyStop = false;
      controlMode   = MODE_MANUAL;
      Serial.println("[ESTOP] Emergency stop cleared. MANUAL mode.");
    } else {
      // Trigger emergency stop
      emergencyStop = true;
      stopAllMotors();
      Serial.println("[ESTOP] EMERGENCY STOP triggered by button!");
    }
  }

  // --- Read accelerometer periodically ---
  if (mpuAvailable && (now - lastSensorRead >= SENSOR_READ_INTERVAL)) {
    readAccelerometer();
    lastSensorRead = now;
  }

  // --- Tilt safety check ---
  if (mpuAvailable) {
    checkTiltSafety();
  }

  // --- Motor timeout safety check ---
  checkMotorTimeouts();

  // --- Execute control logic ---
  if (emergencyStop) {
    stopAllMotors();
  } else {
    if (controlMode == MODE_AUTO) {
      if (now - lastAutoLevelTime >= AUTO_LEVEL_INTERVAL) {
        autoLevelRoutine();
        lastAutoLevelTime = now;
      }
    }
    // Manual mode: motors only controlled by button events
    // (extend/retract logic via BTN_MODE held states could be added here)
    // Currently manual mode leaves motors in their last commanded state
    // and relies on ESTOP button to stop them.
  }

  // --- Periodic status print ---
  if (now - lastStatusPrint >= STATUS_PRINT_INTERVAL) {
    printStatus();
    lastStatusPrint = now;
  }

  delay(10);
}

// ============================================================
// MOTOR INITIALIZATION
// ============================================================
void initMotors() {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    // Direction control pins
    pinMode(motors[i].in1Pin, OUTPUT);
    pinMode(motors[i].in2Pin, OUTPUT);
    digitalWrite(motors[i].in1Pin, LOW);
    digitalWrite(motors[i].in2Pin, LOW);

    // PWM enable channel
    ledcSetup(motors[i].pwmChannel, PWM_FREQ, PWM_RES);
    ledcAttachPin(motors[i].enPin, motors[i].pwmChannel);
    ledcWrite(motors[i].pwmChannel, 0);

    motors[i].state     = MOTOR_STOP;
    motors[i].startTime = 0;
  }
}

// ============================================================
// BUTTON INITIALIZATION
// ============================================================
void initButtons() {
  for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
    // GPIO3 (RX) and GPIO12 can use INPUT_PULLUP
    // GPIO3 is UART RX - avoid heavy serial traffic while using as button
    pinMode(buttons[i].pin, INPUT_PULLUP);
    buttons[i].lastReading      = HIGH;
    buttons[i].stableState      = HIGH;
    buttons[i].lastDebounceTime = 0;
    buttons[i].pressedEvent     = false;
  }
}

// ============================================================
// MPU6050 INITIALIZATION
// ============================================================
bool initMPU() {
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("[ERROR] MPU6050 connection test failed.");
    return false;
  }
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  // +/- 2g
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);  // +/- 250 deg/s
  mpu.setDLPFMode(MPU6050_DLPF_BW_20);             // 20Hz low-pass for stability
  return true;
}

// ============================================================
// SET MOTOR STATE AND SPEED
// ============================================================
void setMotor(uint8_t motorIndex, MotorState state, uint8_t speed) {
  if (motorIndex >= NUM_MOTORS)       return;
  if (!motors[motorIndex].enabled)    return;

  Motor &m = motors[motorIndex];

  switch (state) {
    case MOTOR_STOP:
      digitalWrite(m.in1Pin, LOW);
      digitalWrite(m.in2Pin, LOW);
      ledcWrite(m.pwmChannel, 0);
      m.state     = MOTOR_STOP;
      m.startTime = 0;
      break;

    case MOTOR_EXTEND:
      //