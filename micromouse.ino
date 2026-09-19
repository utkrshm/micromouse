#include <Arduino.h>

// =====================================================
//                    ULTRASONIC PINS
// =====================================================

#define LEFT_TRIG 25
#define LEFT_ECHO 34

#define FRONT_TRIG 26
#define FRONT_ECHO 35

#define RIGHT_TRIG 27
#define RIGHT_ECHO 32

// =====================================================
//                    L298 MOTOR PINS
// =====================================================

// Right motor: L298 OUT1/OUT2
#define RIGHT_IN1 13
#define RIGHT_IN2 12
#define RIGHT_EN 2

// Left motor: L298 OUT3/OUT4
#define LEFT_IN3 18
#define LEFT_IN4 19
#define LEFT_EN 4

// =====================================================
//                    TEST CONFIGURATION
// =====================================================

// Set either value to true if that motor spins backward when commanded forward.
bool LEFT_REVERSED = false;
bool RIGHT_REVERSED = false;

// Start conservatively. Valid range: 0-255.
const int DRIVE_PWM = 120;

// The robot drives continuously after the startup delay.
const bool DRIVE_ON_START = true;
const unsigned long STARTUP_DELAY_MS = 2000;

// =====================================================
//                    PWM CONFIGURATION
// =====================================================

const uint32_t PWM_FREQUENCY = 20000;
const uint8_t PWM_RESOLUTION = 8;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
// Arduino-ESP32 core 3.x uses pin-based LEDC attachment and writes.
#else
const uint8_t RIGHT_PWM_CHANNEL = 0;
const uint8_t LEFT_PWM_CHANNEL = 1;
#endif

// =====================================================
//                    ULTRASONIC CONFIGURATION
// =====================================================

const float SOUND_SPEED_CM_PER_US = 0.0343f;
const unsigned long ECHO_TIMEOUT_US = 25000;
const unsigned long SENSOR_SETTLE_MS = 30;

// =====================================================
//                    PWM FUNCTIONS
// =====================================================

void setupPwm() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(RIGHT_EN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(LEFT_EN, PWM_FREQUENCY, PWM_RESOLUTION);
#else
  ledcSetup(RIGHT_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(LEFT_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(RIGHT_EN, RIGHT_PWM_CHANNEL);
  ledcAttachPin(LEFT_EN, LEFT_PWM_CHANNEL);
#endif
}

void writePwm(int pin, int duty) {
  duty = constrain(duty, 0, 255);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  if (pin == RIGHT_EN) {
    ledcWrite(RIGHT_PWM_CHANNEL, duty);
  } else {
    ledcWrite(LEFT_PWM_CHANNEL, duty);
  }
#endif
}

// speed is signed: positive is forward, negative is backward, zero is stop.
void writeMotor(int in1, int in2, int enablePin, int speed, bool reversed) {
  speed = constrain(speed, -255, 255);

  if (reversed) {
    speed = -speed;
  }

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    writePwm(enablePin, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    writePwm(enablePin, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    writePwm(enablePin, 0);
  }
}

void setLeftMotor(int speed) {
  writeMotor(LEFT_IN3, LEFT_IN4, LEFT_EN, speed, LEFT_REVERSED);
}

void setRightMotor(int speed) {
  writeMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_EN, speed, RIGHT_REVERSED);
}

void stopMotors() {
  setLeftMotor(0);
  setRightMotor(0);
}

void driveForward(int pwm) {
  setLeftMotor(pwm);
  setRightMotor(pwm);
}

// =====================================================
//                    ULTRASONIC FUNCTIONS
// =====================================================

float readDistanceCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  const unsigned long duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) {
    return -1.0f;
  }

  return (duration * SOUND_SPEED_CM_PER_US) / 2.0f;
}

void printSensor(const char *name, float distanceCm) {
  Serial.print(name);
  Serial.print(": ");

  if (distanceCm < 0.0f) {
    Serial.print("NO ECHO");
  } else {
    Serial.print(distanceCm, 1);
    Serial.print(" cm");
  }
}

// =====================================================
//                    SETUP
// =====================================================

void setup() {
  Serial.begin(115200);

  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(LEFT_IN3, OUTPUT);
  pinMode(LEFT_IN4, OUTPUT);
  setupPwm();

  pinMode(LEFT_TRIG, OUTPUT);
  pinMode(LEFT_ECHO, INPUT);
  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);
  pinMode(RIGHT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);

  digitalWrite(LEFT_TRIG, LOW);
  digitalWrite(FRONT_TRIG, LOW);
  digitalWrite(RIGHT_TRIG, LOW);
  stopMotors();

  Serial.println();
  Serial.println(F("===================="));
  Serial.println(F("Robot V0.1 Starting"));
  Serial.println(F("===================="));
  Serial.println(F("Keep the robot lifted during the first motor test."));

  delay(STARTUP_DELAY_MS);
  Serial.println(F("Ready"));
}

// =====================================================
//                    LOOP
// =====================================================

void loop() {
  const float leftDistance = readDistanceCm(LEFT_TRIG, LEFT_ECHO);
  delay(SENSOR_SETTLE_MS);

  const float frontDistance = readDistanceCm(FRONT_TRIG, FRONT_ECHO);
  delay(SENSOR_SETTLE_MS);

  const float rightDistance = readDistanceCm(RIGHT_TRIG, RIGHT_ECHO);

  Serial.println(F("----------------"));
  printSensor("LEFT", leftDistance);
  Serial.println();
  printSensor("FRONT", frontDistance);
  Serial.println();
  printSensor("RIGHT", rightDistance);
  Serial.println();

  if (DRIVE_ON_START) {
    driveForward(DRIVE_PWM);
  } else {
    stopMotors();
  }

  delay(50);
}
