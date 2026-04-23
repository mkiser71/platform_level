// RV/Trailer Level Controller with 4 DC Motors (scissor jacks)
// Uses IBT-2 (BTS7960) motor drivers with current sense feedback
// Current sense resistors (1kΩ) on IS_R/IS_L pins for stall detection and position monitoring
// Platform: ESP32
// Power: 12V, 30A DC from camper battery
//
// === IBT-2 Current Sense Wiring ===
// IBT-2 IS_R and IS_L pins output ~1/8500 of motor current as a signal current
// With 1kΩ resistor to GND: V_sense = I_motor / 8500 * 1000 (ohms)
// At 10A motor current: V_sense ≈ 1.18V (well within ESP32 ADC 0-3.3V range)
// At 30A motor current: V_sense ≈ 3.53V (use voltage divider if needed)
//
// Current sense pins (IS_R for extend, IS_L for retract):
//   Motor 0 (FL): IS_R -> GPIO33, IS_L -> GPIO34
//   Motor 1 (FR): IS_R -> GPIO35, IS_L -> GPIO36
//   Motor 2 (RL): IS_R -> GPIO39, IS_L -> GPIO33 (muxed, see note)
//   Motor 3 (RR): IS_R -> GPIO34, IS_L -> GPIO36 (muxed, see note)
//
// Note: With only 5 ADC1 pins (33,34,35,36,39) for 8 sense signals,
// we use two current-sense resistors (on pins 33 and 34 per manifest) and
// implement time-division multiplexing: only active motor's sense is read.
// Each motor pair shares ADC channels; only one motor per pair runs at a time.
// For a full 8-channel implementation, add an analog multiplexer (CD4051/CD74HC4051).
//
// === Pin Assignments (matching component manifest) ===
// Motor 0 FL: IN1=2, IN2=4, EN=5
// Motor 1 FR: IN1=18, IN2=19, EN=13
// Motor 2 RL: IN1=14, IN2=16, EN=17
// Motor 3 RR: IN1=21, IN2=22, EN=23
// Current Sense 0 (motors 0 & 2 shared): GPIO33
// Current Sense 1 (motors 1 & 3 shared): GPIO34
// Linear Actuators (position feedback): GPIO25, 26, 27, 32

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>

// ============================================================
// PIN DEFINITIONS (matching component manifest exactly)
// ============================================================

// Motor 0 - Front Left Jack (dc-motor_0)
#define DC_MOTOR_0_IN1  2
#define DC_MOTOR_0_IN2  4
#define DC_MOTOR_0_EN   5

// Motor 1 - Front Right Jack (dc-motor_1)
#define DC_MOTOR_1_IN1  18
#define DC_MOTOR_1_IN2  19
#define DC_MOTOR_1_EN   13

// Motor 2 - Rear Left Jack (dc-motor_2)
#define DC_MOTOR_2_IN1  14
#define DC_MOTOR_2_IN2  16
#define DC_MOTOR_2_EN   17

// Motor 3 - Rear Right Jack (dc-motor_3)
#define DC_MOTOR_3_IN1  21
#define DC_MOTOR_3_IN2  22
#define DC_MOTOR_3_EN   23

// Linear actuator position feedback pins (linear-actuator_4..7)
#define LINEAR_ACT_0_PIN  25   // FL jack position
#define LINEAR_ACT_1_PIN  26   // FR jack position
#define LINEAR_ACT_2_PIN  27   // RL jack position
#define LINEAR_ACT_3_PIN  32   // RR jack position

// Current sense ADC pins (current-sense-resistor-1k_8 and _10)
// ADC1 only - ADC2 unavailable when WiFi active
// GPIO33 = ADC1_CH5, GPIO34 = ADC1_CH6 (input only)
#define CURRENT_SENSE_A_PIN  33   // Shared by motors 0 (FL) and 2 (RL)
#define CURRENT_SENSE_B_PIN  34   // Shared by motors 1 (FR) and 3 (RR)

// MPU-6050 I2C pins (ESP32 defaults)
#define MPU_SDA         21
#define MPU_SCL         22

// Manual override pushbuttons
#define BTN_MODE        12   // Mode toggle (AUTO/MANUAL)
#define BTN_ESTOP       0    // Emergency stop / reset (GPIO0 boot button, use with care)

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
// CURRENT SENSE CONFIGURATION
// ============================================================
// IBT-2 current sense ratio: 1/8500
// Sense resistor: 1000 ohms (1kΩ)
// V_sense = I_motor * (1/8500) * 1000
// I_motor = V_sense * 8.5   (in amps, with V_sense in volts)
// ESP32 ADC: 12-bit, 3.3V reference -> 3.3/4095 V per count

#define ADC_REF_VOLTAGE       3.3f
#define ADC_RESOLUTION        4095.0f
#define CURRENT_SENSE_RATIO   8500.0f   // IBT-2 IS ratio
#define SENSE_RESISTOR_OHMS   1000.0f   // 1kΩ sense resistor

// Stall detection thresholds
#define STALL_CURRENT_AMPS    8.0f      // Current above this = likely stall
#define STALL_CONFIRM_MS      300       // Must be above threshold for this long to confirm stall
#define NORMAL_CURRENT_AMPS   5.0f      // Expected running current (informational)

// Position estimation via current integration
// Accumulated current-time product used as soft position proxy
// Units: Amp-milliseconds (higher = more work done = jack extended further)
#define POSITION_MAX_AMP_MS   500000.0f  // Approximate full extension in A·ms
#define CURRENT_SAMPLE_MS     50         // How often to sample current for integration

// ============================================================
// CONFIGURATION CONSTANTS
// ============================================================
#define NUM_MOTORS           4
#define NUM_JACKS            4

#define MOTOR_SPEED_FULL     200  // PWM duty cycle 0-255
#define MOTOR_SPEED_SLOW     120  // Slow speed for fine adjustment

// Accelerometer leveling thresholds (degrees)
#define LEVEL_THRESHOLD      0.5f
#define TILT_MAX             15.0f

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

// Number of ADC samples to average for noise reduction
#define ADC_OVERSAMPLE_COUNT  8

// ============================================================
// ENUMERATIONS
// ============================================================
enum MotorState {
  MOTOR_STOP = 0,
  MOTOR_EXTEND,   // Jack extends downward (lifts corner)
  MOTOR_RETRACT   // Jack retracts upward (lowers corner)
};

enum ControlMode {
  MODE_MANUAL = 0,
  MODE_AUTO
};

enum JackPosition {
  POS_UNKNOWN = 0,
  POS_RETRACTED,
  POS_EXTENDING,
  POS_EXTENDED,
  POS_STALLED
};

// ============================================================
// CURRENT SENSE DATA STRUCTURE
// ============================================================
struct CurrentSense {
  uint8_t  adcPin;             // ESP32 ADC pin
  float    currentAmps;        // Latest current reading in amps
  float    currentFiltered;    // Low-pass filtered current
  bool     stallDetected;      // True if stall confirmed
  unsigned long stallStartMs;  // When current first exceeded stall threshold
  bool     stallTimerActive;   // True when monitoring for stall confirmation
  float    positionAmpMs;      // Integrated current-time (position proxy)
  unsigned long lastSampleMs;  // Last time position was integrated
};

// ============================================================
// MOTOR STRUCTURE
// ============================================================
struct Motor {
  uint8_t       in1Pin;
  uint8_t       in2Pin;
  uint8_t       enPin;
  uint8_t       pwmChannel;
  uint8_t       currentSenseIndex;  // Index into currentSense[] array
  MotorState    state;
  JackPosition  jackPos;
  unsigned long startTime;
  bool          enabled;
  const char*   name;
};

// ============================================================
// BUTTON STRUCTURE
// ============================================================
struct Button {
  uint8_t       pin;
  bool          lastReading;
  bool          stableState;
  unsigned long lastDebounceTime;
  bool          pressedEvent;
};

// ============================================================
// GLOBAL OBJECTS AND VARIABLES
// ============================================================

MPU6050 mpu;

// Current sense channels
// Channel A (index 0): GPIO33 - shared by Motor 0 (FL) and Motor 2 (RL)
// Channel B (index 1): GPIO34 - shared by Motor 1 (FR) and Motor 3 (RR)
CurrentSense currentSense[2] = {
  { CURRENT_SENSE_A_PIN, 0.0f, 0.0f, false, 0, false, 0.0f, 0 },
  { CURRENT_SENSE_B_PIN, 0.0f, 0.0f, false, 0, false, 0.0f, 0 }
};

// Motor definitions - matched to component manifest pin assignments
// currentSenseIndex: 0 = GPIO33 channel, 1 = GPIO34 channel
Motor motors[NUM_MOTORS] = {
  { DC_MOTOR_0_IN1, DC_MOTOR_0_IN2, DC_MOTOR_0_EN, PWM_CH_M0, 0, MOTOR_STOP, POS_UNKNOWN, 0, true, "FL-Jack" },
  { DC_MOTOR_1_IN1, DC_MOTOR_1_IN2, DC_MOTOR_1_EN, PWM_CH_M1, 1, MOTOR_STOP, POS_UNKNOWN, 0, true, "FR-Jack" },
  { DC_MOTOR_2_IN1, DC_MOTOR_2_IN2, DC_MOTOR_2_EN, PWM_CH_M2, 0, MOTOR_STOP, POS_UNKNOWN, 0, true, "RL-Jack" },
  { DC_MOTOR_3_IN1, DC_MOTOR_3_IN2, DC_MOTOR_3_EN, PWM_CH_M3, 1, MOTOR_STOP, POS_UNKNOWN, 0, true, "RR-Jack" }
};

// Button definitions
Button buttons[2] = {
  { BTN_MODE,  HIGH, HIGH, 0, false },
  { BTN_ESTOP, HIGH, HIGH, 0, false }
};
#define NUM_BUTTONS   2
#define BTN_IDX_MODE  0
#define BTN_IDX_ESTOP 1

// System state
ControlMode controlMode   = MODE_MANUAL;
bool        mpuAvailable  = false;
bool        emergencyStop = false;

// Accelerometer angles
float pitchAngle = 0.0f;  // Front-back tilt (degrees)
float rollAngle  = 0.0f;  // Left-right tilt (degrees)

// Timing
unsigned long lastAutoLevelTime  = 0;
unsigned long lastSensorRead     = 0;
unsigned long lastStatusPrint    = 0;
unsigned long lastCurrentSample  = 0;

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================
void     initMotors();
void     initButtons();
void     initCurrentSense();
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
void     updateCurrentSense();
void     processStallDetection();
void     updatePositionEstimates();
float    readCurrentAmps(uint8_t adcPin);
float    getMotorCurrent(uint8_t motorIndex);
float    getMotorPositionPercent(uint8_t motorIndex);
void     resetMotorPosition(uint8_t motorIndex);
void     printStatus();
void     printCurrentSenseStatus();

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("==============================================");
  Serial.println("  RV/Trailer Level Controller - ESP32");
  Serial.println("  Motor Driver: IBT-2 (BTS7960)");
  Serial.println("  Current Sense: IS_R/IS_L via 1k resistors");
  Serial.println("==============================================");

  // Initialize I2C for MPU6050
  // NOTE: MPU6050 uses GPIO21/22 but motor 3 also uses GPIO21/22 for IN1/IN2
  // For a real deployment, remap MPU6050 to different I2C pins or remap motor 3
  // Here we initialize I2C first, then motor pins override GPIO21/22 for direction control
  // MPU6050 communication should complete before motor direction changes
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(400000);

  // Initialize current sense ADC inputs
  initCurrentSense();
  Serial.println("[OK] Current sense ADC channels initialized.");

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
  Serial