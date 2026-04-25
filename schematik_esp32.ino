// Camper/RV Scissor Jack Auto-Leveling Controller
// Uses 4x IBT-2 motor controllers and MPU-6050 accelerometer
// Platform: ESP32
// Supply: 12V DC, 30A max per motor controller

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================

// Motor 0 (Front-Left Jack) - IBT-2 Controller
#define MOTOR0_IN1  5
#define MOTOR0_IN2  18
#define MOTOR0_EN   19

// Motor 1 (Front-Right Jack) - IBT-2 Controller
#define MOTOR1_IN1  13
#define MOTOR1_IN2  14
#define MOTOR1_EN   16

// Motor 2 (Rear-Left Jack) - IBT-2 Controller
#define MOTOR2_IN1  17
#define MOTOR2_IN2  23
#define MOTOR2_EN   25

// Motor 3 (Rear-Right Jack) - IBT-2 Controller
#define MOTOR3_IN1  26
#define MOTOR3_IN2  27
#define MOTOR3_EN   32

// MPU-6050 Pins
#define MPU_SCL     22
#define MPU_SDA     21
#define MPU_INT     2
#define MPU_AD0     4

// Auxiliary Data Pins
#define BUCK_DATA   33
#define FUSE_DATA   34  // Input-only pin on ESP32

// ============================================================
// LEDC PWM CHANNEL ASSIGNMENTS
// ============================================================
#define MOTOR0_PWM_CH  0
#define MOTOR1_PWM_CH  1
#define MOTOR2_PWM_CH  2
#define MOTOR3_PWM_CH  3

#define PWM_FREQ       5000
#define PWM_RESOLUTION 8    // 8-bit: 0-255

// ============================================================
// LEVELING CONSTANTS
// ============================================================
#define LEVEL_TOLERANCE_DEG   0.5f    // Degrees considered "level"
#define COARSE_TOLERANCE_DEG  2.0f    // Degrees for coarse correction
#define MOTOR_SPEED_FULL      200     // PWM value for full speed (0-255)
#define MOTOR_SPEED_SLOW      120     // PWM value for slow/fine adjustment
#define SENSOR_SAMPLES        20      // Samples to average for stability
#define SAMPLE_DELAY_MS       10      // Delay between samples (ms)
#define MOTOR_RUN_TIME_MS     300     // Duration to run motor per correction step (ms)
#define SETTLE_TIME_MS        800     // Time to wait for platform to settle after movement
#define LOOP_DELAY_MS         200     // Main loop delay

// Max correction attempts before giving up
#define MAX_CORRECTION_ATTEMPTS 50

// ============================================================
// MOTOR DIRECTION DEFINITIONS
// ============================================================
#define DIR_STOP    0
#define DIR_EXTEND  1   // Jack extends (raises that corner)
#define DIR_RETRACT 2   // Jack retracts (lowers that corner)

// ============================================================
// MOTOR STRUCTURE
// ============================================================
struct Motor {
  uint8_t in1;
  uint8_t in2;
  uint8_t en;
  uint8_t pwmChannel;
  const char* name;
};

// Motor layout:
//   Motor 0 = Front-Left
//   Motor 1 = Front-Right
//   Motor 2 = Rear-Left
//   Motor 3 = Rear-Right

Motor motors[4] = {
  { MOTOR0_IN1, MOTOR0_IN2, MOTOR0_EN, MOTOR0_PWM_CH, "Front-Left"  },
  { MOTOR1_IN1, MOTOR1_IN2, MOTOR1_EN, MOTOR1_PWM_CH, "Front-Right" },
  { MOTOR2_IN1, MOTOR2_IN2, MOTOR2_EN, MOTOR2_PWM_CH, "Rear-Left"   },
  { MOTOR3_IN1, MOTOR3_IN2, MOTOR3_EN, MOTOR3_PWM_CH, "Rear-Right"  }
};

// ============================================================
// GLOBAL OBJECTS & VARIABLES
// ============================================================
Adafruit_MPU6050 mpu;

bool mpuReady = false;
int correctionAttempts = 0;

// Leveling state machine
enum LevelState {
  STATE_INIT,
  STATE_READ_SENSOR,
  STATE_EVALUATE,
  STATE_CORRECTING,
  STATE_SETTLING,
  STATE_LEVELED,
  STATE_ERROR
};

LevelState currentState = STATE_INIT;

// ============================================================
// FUNCTION PROTOTYPES
// ============================================================
void initMotors();
void setMotor(uint8_t idx, uint8_t direction, uint8_t speed);
void stopAllMotors();
bool readMPUAngles(float &rollDeg, float &pitchDeg);
void performLevelingStep(float roll, float pitch);
void printAngles(float roll, float pitch);

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("===========================================");
  Serial.println("  Camper/RV Scissor Jack Leveling System  ");
  Serial.println("===========================================");

  // Initialize motor driver pins and PWM
  initMotors();
  stopAllMotors();
  Serial.println("[Motor] All motors initialized and stopped.");

  // Set AD0 low (MPU-6050 address = 0x68)
  pinMode(MPU_AD0, OUTPUT);
  digitalWrite(MPU_AD0, LOW);

  // Initialize MPU-6050
  Wire.begin(MPU_SDA, MPU_SCL);
  Serial.println("[MPU] Initializing MPU-6050...");

  if (!mpu.begin()) {
    Serial.println("[MPU] ERROR: MPU-6050 not found! Check wiring.");
    mpuReady = false;
    currentState = STATE_ERROR;
  } else {
    Serial.println("[MPU] MPU-6050 found and initialized.");
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpuReady = true;
    currentState = STATE_READ_SENSOR;
    Serial.println("[SYS] Leveling system ready. Starting leveling process...");
  }

  // Setup MPU interrupt pin (input)
  pinMode(MPU_INT, INPUT);

  // BUCK_DATA and FUSE_DATA are input-only / monitoring pins
  // GPIO34 is input-only on ESP32
  pinMode(FUSE_DATA, INPUT);

  delay(500);
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  switch (currentState) {

    case STATE_INIT:
      // Re-attempt initialization
      currentState = STATE_READ_SENSOR;
      break;

    case STATE_READ_SENSOR: {
      float roll = 0.0f, pitch = 0.0f;
      if (!mpuReady) {
        Serial.println("[ERROR] MPU not ready.");
        currentState = STATE_ERROR;
        break;
      }
      if (readMPUAngles(roll, pitch)) {
        printAngles(roll, pitch);
        currentState = STATE_EVALUATE;
        // Store angles for evaluation
        // Pass directly to evaluate step
        float absRoll  = fabs(roll);
        float absPitch = fabs(pitch);

        if (absRoll <= LEVEL_TOLERANCE_DEG && absPitch <= LEVEL_TOLERANCE_DEG) {
          if (currentState != STATE_LEVELED) {
            Serial.println("[LEVEL] Vehicle is LEVEL! All jacks stopped.");
          }
          currentState = STATE_LEVELED;
          stopAllMotors();
          correctionAttempts = 0;
        } else {
          currentState = STATE_CORRECTING;
          performLevelingStep(roll, pitch);
          currentState = STATE_SETTLING;
        }
      } else {
        Serial.println("[MPU] Failed to read sensor data.");
        delay(500);
      }
      break;
    }

    case STATE_EVALUATE:
      // Handled inline in STATE_READ_SENSOR
      currentState = STATE_READ_SENSOR;
      break;

    case STATE_CORRECTING:
      // Correction handled in STATE_READ_SENSOR
      currentState = STATE_SETTLING;
      break;

    case STATE_SETTLING:
      Serial.println("[SYS] Settling...");
      delay(SETTLE_TIME_MS);
      correctionAttempts++;
      if (correctionAttempts >= MAX_CORRECTION_ATTEMPTS) {
        Serial.println("[ERROR] Max correction attempts reached. Stopping.");
        stopAllMotors();
        currentState = STATE_ERROR;
      } else {
        currentState = STATE_READ_SENSOR;
      }
      break;

    case STATE_LEVELED:
      // Periodically re-check level in case of drift
      delay(2000);
      Serial.println("[LEVEL] Re-checking level...");
      currentState = STATE_READ_SENSOR;
      break;

    case STATE_ERROR:
      stopAllMotors();
      Serial.println("[ERROR] System in error state. Check hardware. Retrying in 5s...");
      delay(5000);
      // Attempt recovery
      if (!mpuReady) {
        if (mpu.begin()) {
          mpuReady = true;
          mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
          mpu.setGyroRange(MPU6050_RANGE_250_DEG);
          mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
          Serial.println("[MPU] Recovered. Resuming.");
          correctionAttempts = 0;
          currentState = STATE_READ_SENSOR;
        }
      } else {
        correctionAttempts = 0;
        currentState = STATE_READ_SENSOR;
      }
      break;

    default:
      currentState = STATE_READ_SENSOR;
      break;
  }

  delay(LOOP_DELAY_MS);
}

// ============================================================
// INITIALIZE MOTORS
// ============================================================
void initMotors() {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(motors[i].in1, OUTPUT);
    pinMode(motors[i].in2, OUTPUT);

    // Setup LEDC PWM channel for enable pin
    ledcSetup(motors[i].pwmChannel, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(motors[i].en, motors[i].pwmChannel);

    // Start with motor stopped
    digitalWrite(motors[i].in1, LOW);
    digitalWrite(motors[i].in2, LOW);
    ledcWrite(motors[i].pwmChannel, 0);
  }
}

// ============================================================
// SET INDIVIDUAL MOTOR
// direction: DIR_STOP, DIR_EXTEND, DIR_RETRACT
// speed: 0-255 PWM value
// ============================================================
void setMotor(uint8_t idx, uint8_t direction, uint8_t speed) {
  if (idx > 3) return;

  switch (direction) {
    case DIR_EXTEND:
      digitalWrite(motors[idx].in1, HIGH);
      digitalWrite(motors[idx].in2, LOW);
      ledcWrite(motors[idx].pwmChannel, speed);
      Serial.printf("[MOTOR] %s: EXTENDING (speed=%d)\n", motors[idx].name, speed);
      break;

    case DIR_RETRACT:
      digitalWrite(motors[idx].in1, LOW);
      digitalWrite(motors[idx].in2, HIGH);
      ledcWrite(motors[idx].pwmChannel, speed);
      Serial.printf("[MOTOR] %s: RETRACTING (speed=%d)\n", motors[idx].name, speed);
      break;

    case DIR_STOP:
    default:
      digitalWrite(motors[idx].in1, LOW);
      digitalWrite(motors[idx].in2, LOW);
      ledcWrite(motors[idx].pwmChannel, 0);
      break;
  }
}

// ============================================================
// STOP ALL MOTORS
// ============================================================
void stopAllMotors() {
  for (uint8_t i = 0; i < 4; i++) {
    setMotor(i, DIR_STOP, 0);
  }
  Serial.println("[MOTOR] All motors STOPPED.");
}

// ============================================================
// READ MPU-6050 ANGLES (AVERAGED)
// Returns roll (side-to-side) and pitch (front-to-back) in degrees
// ============================================================
bool readMPUAngles(float &rollDeg, float &pitchDeg) {
  if (!mpuReady) return false;

  float accX_sum = 0, accY_sum = 0, accZ_sum = 0;

  for (int i = 0; i < SENSOR_SAMPLES; i++) {
    sensors_event_t accelEvent, gyroEvent, tempEvent;
    if (!mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent)) {
      return false;
    }
    accX_sum += accelEvent.acceleration.x;
    accY_sum += accelEvent.acceleration.y;
    accZ_sum += accelEvent.acceleration.z;
    delay(SAMPLE_DELAY_MS);
  }

  float ax = accX_sum / SENSOR_SAMPLES;
  float ay = accY_sum / SENSOR_SAMPLES;
  float az = accZ_sum / SENSOR_SAMPLES;

  // Calculate roll and pitch from accelerometer data
  // Roll  = rotation around X axis (left-right tilt)
  // Pitch = rotation around Y axis (front-back tilt)
  rollDeg  = atan2(ay, az) * 180.0f / PI;
  pitchDeg = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;

  return true;
}

// ============================================================
// PERFORM ONE LEVELING CORRECTION STEP
// Logic:
//   Roll  > 0 (tilting right) -> extend Front-Left + Rear-Left (motors 0,2)
//                              OR retract Front-Right + Rear-Right (motors 1,3)
//   Roll  < 0 (tilting left)  -> extend Front-Right + Rear-Right (motors 1,3)
//                              OR retract Front-Left + Rear-Left (motors 0,2)
//   Pitch > 0 (nose up/rear down) -> extend Front-Left + Front-Right (motors 0,1)
//                                 OR retract Rear-Left + Rear-Right (motors 2,3)
//   Pitch < 0 (nose down/rear up) -> extend Rear-Left + Rear-Right (motors 2,3)
//                                 OR retract Front-Left + Front-Right (motors 0,1)
// Strategy: Extend the low side (raises that corner)
// ============================================================
void performLevelingStep(float roll, float pitch) {
  stopAllMotors();

  float absRoll  = fabs(roll);
  float absPitch = fabs(pitch