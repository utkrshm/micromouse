# Transcript: Optimize Robot Component Placement (ChatGPT share, updated 2026-09-19)

## User

I have 2 BO motors, ultrasonic sensors at 45 degree in the front like this (the image), a L298 motor driver and LiPo batteries... I need you to give me the code for navigation in an extremely tight turning space for the robot. Reason with me together, what is the best strategy for movement?

This is the code I have so far in Arduino IDE for getting readings of the 3 ultrasonic sensors at the front:

```
#include<Arduino.h>

#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701
#define ULTRASONIC_THRESHOLD 15

const int leftTrigPin = 25;
const int leftEchoPin = 34;
const int frontTrigPin = 26;
const int frontEchoPin = 35;
const int rightTrigPin = 27;
const int rightEchoPin = 32;

void showSensorReading(const char* name, int echoPin, int trigPin) {
  long duration;
  float distanceCm;
  float distanceInch;

  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance
  distanceCm = duration * SOUND_SPEED/2;

  // Convert to inches
  distanceInch = distanceCm * CM_TO_INCH;

  // if (distanceCm < ULTRASONIC_THRESHOLD) {
  Serial.print("Sensor direction: ");
  Serial.print(name);
  Serial.print("; Cm distance: ");
  Serial.print(distanceCm);
  Serial.print("; Inch distance: ");
  Serial.print(distanceInch);
  Serial.print("\n");
  // }
  // else {
  //   Serial.print("Sensor in direction " + name + "shows distance greater than 15cm");
  // }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(leftTrigPin, OUTPUT);
  pinMode(frontTrigPin, OUTPUT);
  pinMode(rightTrigPin, OUTPUT);

  pinMode(leftEchoPin, INPUT);
  pinMode(frontEchoPin, INPUT);
  pinMode(rightEchoPin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  showSensorReading("left", leftEchoPin, leftTrigPin);
  delay(100);
  showSensorReading("front", frontEchoPin, frontTrigPin);
  delay(100);
  showSensorReading("right", rightEchoPin, rightTrigPin);
  delay(1000);
}
```

## Assistant

Great. Now we have the **direction control pins**.

Your L298 mapping is:

```
L298             ESP32

OUT1/OUT2  → RIGHT MOTOR

IN1        → D13
IN2        → D12

OUT3/OUT4  → LEFT MOTOR

IN3        → D18
IN4        → D19
```

So our motor abstraction becomes:

```cpp
// Right motor
#define RIGHT_IN1 13
#define RIGHT_IN2 12

// Left motor
#define LEFT_IN1 18
#define LEFT_IN2 19
```

The remaining question is **ENA and ENB**.

---

## Case 1 — ENA/ENB jumpers still installed

Many L298 modules come with:

```
ENA [jumper]
IN1
IN2

ENB [jumper]
IN3
IN4
```

If those jumpers are still there:

- Motor channels are always enabled.
- We cannot PWM speed-control through ENA/ENB.
- We can only do:

```
Forward
Backward
Stop
```

using IN pins.

For maze solving, this is **not ideal** because we need:

- slow turns
- controlled approach
- speed reduction before junctions

---

## Case 2 — ENA/ENB connected to ESP32

Then we can do proper control:

```
L298

ENA → ESP32 GPIO ?
IN1 → D13
IN2 → D12

ENB → ESP32 GPIO ?
IN3 → D18
IN4 → D19
```

and our code works as intended:

```cpp
analogWrite(ENA, pwm);
analogWrite(ENB, pwm);
```

---

## For your `.ino`, update this section:

Replace:

```cpp
#define RIGHT_IN1  -1
#define RIGHT_IN2  -1
#define RIGHT_EN   -1

#define LEFT_IN1   -1
#define LEFT_IN2   -1
#define LEFT_EN    -1
```

with:

```cpp
#define RIGHT_IN1 13
#define RIGHT_IN2 12

#define LEFT_IN1 18
#define LEFT_IN2 19
```

For now leave:

```cpp
#define RIGHT_EN -1
#define LEFT_EN  -1
```

until we confirm ENA/ENB.

---

## One more important correction

The earlier motor function assumes we have PWM pins:

```cpp
analogWrite(en, speed);
```

If ENA/ENB are not connected, that code will fail.

So the next version should support **both modes**:

```
PWM mode:
ENA/ENB connected
        ↓
speed control

Enable mode:
jumpers installed
        ↓
full speed only
```

We'll make the motor driver layer automatically handle this.

---

## Current known wiring:

### Ultrasonic

```
LEFT:
TRIG D25
ECHO D34

FRONT:
TRIG D26
ECHO D35

RIGHT:
TRIG D27
ECHO D32
```

### L298

```
RIGHT MOTOR:
OUT1/OUT2

IN1 → D13
IN2 → D12

LEFT MOTOR:
OUT3/OUT4

IN3 → D18
IN4 → D19
```

---

I need just two more pieces before I give you the corrected final `.ino`:

1. **Are ENA and ENB jumpers currently installed on your L298 board?**

    - Yes/no
2. If no:

    - ENA → which ESP32 pin?
    - ENB → which ESP32 pin?

After that, I'll give you the cleaned-up Version 0.2 code with your actual pins.

## User

ENA D2&#x20;
ENB D4

## Assistant

Excellent. Now we have the complete L298 wiring.

Your motor driver mapping is:

```
                 L298

             RIGHT MOTOR
             OUT1 / OUT2

ESP32:
IN1  → D13
IN2  → D12
ENA  → D2 (PWM)

             LEFT MOTOR
             OUT3 / OUT4

ESP32:
IN3  → D18
IN4  → D19
ENB  → D4 (PWM)
```

This is actually a good setup because now we have **independent PWM speed control** for both motors. That means our original plan works:

- slow approach to junctions ✅
- controlled turns ✅
- straight-line correction later ✅
- different left/right motor compensation ✅

---

## Update your motor configuration section to:

```cpp
// ======================================================
//                   L298 MOTOR PINS
// ======================================================

// Right motor (OUT1/OUT2)
#define RIGHT_IN1 13
#define RIGHT_IN2 12
#define RIGHT_EN  2

// Left motor (OUT3/OUT4)
#define LEFT_IN3 18
#define LEFT_IN4 19
#define LEFT_EN  4
```

---

## One important ESP32 note

On ESP32, `analogWrite()` works in Arduino core versions, but internally it uses LEDC PWM. For reliability, I would eventually migrate the motor code to explicit LEDC channels:

Example:

```cpp
ledcAttachPin(RIGHT_EN, 0);
ledcAttachPin(LEFT_EN, 1);

ledcSetup(0, 20000, 8);
ledcSetup(1, 20000, 8);
```

Then:

```cpp
ledcWrite(0, pwm);
ledcWrite(1, pwm);
```

This gives stable motor PWM at 20 kHz and avoids weird behavior.

For the first prototype, `analogWrite()` may work, but I would prefer LEDC for the final robot.

---

# Updated motor abstraction

With your pins, the logical control becomes:

```cpp
void setLeftMotor(int speed)
{
    motorWrite(
        LEFT_IN3,
        LEFT_IN4,
        LEFT_EN,
        speed,
        LEFT_REVERSED
    );
}

void setRightMotor(int speed)
{
    motorWrite(
        RIGHT_IN1,
        RIGHT_IN2,
        RIGHT_EN,
        speed,
        RIGHT_REVERSED
    );
}
```

Everything above this layer stays unchanged.

The navigation system will only call:

```cpp
driveForward(120);

pivotLeft(100);

pivotRight(100);

stopMotors();
```

It won't know anything about D13/D12/D18/D19.

---

# Now our hardware layer is complete

Current wiring:

## Ultrasonic

```
Left:
TRIG 25
ECHO 34

Front:
TRIG 26
ECHO 35

Right:
TRIG 27
ECHO 32
```

## L298

```
RIGHT MOTOR
OUT1/OUT2

IN1 → GPIO13
IN2 → GPIO12
ENA → GPIO2

LEFT MOTOR
OUT3/OUT4

IN3 → GPIO18
IN4 → GPIO19
ENB → GPIO4
```

---

# Before I give the cleaned Version 0.2 `.ino`, I need one last hardware detail:

What battery voltage are you using?

Specifically:

- **1S LiPo (3.7V nominal, 4.2V full)?**
- **2S LiPo (7.4V nominal, 8.4V full)?**
- something else?

This affects the first PWM values.

For example, with BO motors:

- 1S + L298 may barely move or turn slowly.
- 2S + L298 may have enough torque but require lower PWM.

Also tell me the BO motor rating if written on it (for example **3–6V**, **6–12V**, etc.).

Then I'll give you the corrected `.ino` with:

- your exact pins
- ESP32 LEDC PWM
- motor polarity config
- ultrasonic module
- sensor filtering
- turn-commitment state machine.

## User

Give me the v0.1 code for checking the straight forward motion, and log the sensor readings from every ultrasonic sensor. A final runnable .ino file with include headers of .cpp and .h (if at all that's possible)

## Assistant

Good. For **V0.1**, we should keep it extremely simple.

Goal:

1. Verify:

    - Both motors move forward
    - Robot direction is correct
    - PWM speed works
2. Verify:

    - All 3 ultrasonic sensors are working
    - Readings are logged continuously
3. Do **not** add:

    - maze logic
    - turns
    - filtering
    - PID
    - state machine

This is our hardware bring-up stage.

---

## File structure

For Arduino IDE, you have two options:

### Option A (recommended for now)

Single `.ino` file.

Arduino IDE handles this easiest.

```
Robot_V0_1/
└── Robot_V0_1.ino
```

### Option B

Multiple files:

```
Robot_V0_1/
│
├── Robot_V0_1.ino
├── Motor.cpp
├── Motor.h
├── Ultrasonic.cpp
└── Ultrasonic.h
```

This is cleaner for the final robot, but for **first hardware testing**, I prefer one file so we eliminate compilation issues.

After motors + sensors are verified, we'll split it.

---

# Robot_V0_1.ino

Copy this directly into Arduino IDE.

```cpp
#include <Arduino.h>

// =====================================================
//                ULTRASONIC PINS
// =====================================================

#define LEFT_TRIG   25
#define LEFT_ECHO   34

#define FRONT_TRIG  26
#define FRONT_ECHO  35

#define RIGHT_TRIG  27
#define RIGHT_ECHO  32

// =====================================================
//                L298 MOTOR PINS
// =====================================================

// Right motor
#define RIGHT_IN1 13
#define RIGHT_IN2 12
#define RIGHT_EN  2

// Left motor
#define LEFT_IN3 18
#define LEFT_IN4 19
#define LEFT_EN  4

// =====================================================
//                MOTOR CONFIG
// =====================================================

// Change these after testing

bool LEFT_REVERSED  = false;
bool RIGHT_REVERSED = false;

// Start conservative
int DRIVE_PWM = 120;

// =====================================================
//                PWM CONFIG
// =====================================================

#define RIGHT_PWM_CHANNEL 0
#define LEFT_PWM_CHANNEL  1

#define PWM_FREQ 20000
#define PWM_RESOLUTION 8

// =====================================================
//                ULTRASONIC CONFIG
// =====================================================

#define SOUND_SPEED 0.0343

// =====================================================
//                MOTOR FUNCTIONS
// =====================================================

void setupPWM()
{
    ledcSetup(
        RIGHT_PWM_CHANNEL,
        PWM_FREQ,
        PWM_RESOLUTION
    );

    ledcSetup(
        LEFT_PWM_CHANNEL,
        PWM_FREQ,
        PWM_RESOLUTION
    );

    ledcAttachPin(
        RIGHT_EN,
        RIGHT_PWM_CHANNEL
    );

    ledcAttachPin(
        LEFT_EN,
        LEFT_PWM_CHANNEL
    );
}

void writeMotor(
    int in1,
    int in2,
    int pwmChannel,
    int speed,
    bool reversed
)
{

    speed = constrain(
        speed,
        -255,
        255
    );

    if(reversed)
        speed = -speed;

    if(speed > 0)
    {
        digitalWrite(in1,HIGH);
        digitalWrite(in2,LOW);

        ledcWrite(
            pwmChannel,
            speed
        );
    }

    else if(speed < 0)
    {
        digitalWrite(in1,LOW);
        digitalWrite(in2,HIGH);

        ledcWrite(
            pwmChannel,
            -speed
        );
    }

    else
    {
        digitalWrite(in1,LOW);
        digitalWrite(in2,LOW);

        ledcWrite(
            pwmChannel,
            0
        );
    }

}

void setLeftMotor(int speed)
{
    writeMotor(
        LEFT_IN3,
        LEFT_IN4,
        LEFT_PWM_CHANNEL,
        speed,
        LEFT_REVERSED
    );
}

void setRightMotor(int speed)
{
    writeMotor(
        RIGHT_IN1,
        RIGHT_IN2,
        RIGHT_PWM_CHANNEL,
        speed,
        RIGHT_REVERSED
    );
}

void stopMotors()
{
    setLeftMotor(0);
    setRightMotor(0);
}

void driveForward(int pwm)
{
    setLeftMotor(pwm);
    setRightMotor(pwm);
}

void driveBackward(int pwm)
{
    setLeftMotor(-pwm);
    setRightMotor(-pwm);
}

// =====================================================
//                ULTRASONIC FUNCTIONS
// =====================================================

float readDistance(
    int trigPin,
    int echoPin
)
{

    digitalWrite(
        trigPin,
        LOW
    );

    delayMicroseconds(2);

    digitalWrite(
        trigPin,
        HIGH
    );

    delayMicroseconds(10);

    digitalWrite(
        trigPin,
        LOW
    );

    long duration =
        pulseIn(
            echoPin,
            HIGH,
            25000
        );

    if(duration == 0)
    {
        return -1;
    }

    float distance =
        duration *
        SOUND_SPEED /
        2.0;

    return distance;
}

void printSensor(
    const char* name,
    float distance
)
{

    Serial.print(name);
    Serial.print(": ");

    if(distance < 0)
    {
        Serial.print("NO ECHO");
    }
    else
    {
        Serial.print(distance);
        Serial.print(" cm");
    }

}

// =====================================================
//                SETUP
// =====================================================

void setup()
{

    Serial.begin(115200);

    Serial.println();
    Serial.println("====================");
    Serial.println("Robot V0.1 Starting");
    Serial.println("====================");

    // Motor pins

    pinMode(
        RIGHT_IN1,
        OUTPUT
    );

    pinMode(
        RIGHT_IN2,
        OUTPUT
    );

    pinMode(
        LEFT_IN3,
        OUTPUT
    );

    pinMode(
        LEFT_IN4,
        OUTPUT
    );

    setupPWM();

    // Ultrasonic pins

    pinMode(
        LEFT_TRIG,
        OUTPUT
    );

    pinMode(
        LEFT_ECHO,
        INPUT
    );

    pinMode(
        FRONT_TRIG,
        OUTPUT
    );

    pinMode(
        FRONT_ECHO,
        INPUT
    );

    pinMode(
        RIGHT_TRIG,
        OUTPUT
    );

    pinMode(
        RIGHT_ECHO,
        INPUT
    );

    stopMotors();

    delay(1000);

    Serial.println("Ready");

}

// =====================================================
//                LOOP
// =====================================================

void loop()
{

    // -----------------------------
    // Read ultrasonic sensors
    // -----------------------------

    float leftDistance =
        readDistance(
            LEFT_TRIG,
            LEFT_ECHO
        );

    delay(30);

    float frontDistance =
        readDistance(
            FRONT_TRIG,
            FRONT_ECHO
        );

    delay(30);

    float rightDistance =
        readDistance(
            RIGHT_TRIG,
            RIGHT_ECHO
        );

    Serial.println("----------------");

    printSensor(
        "LEFT",
        leftDistance
    );

    Serial.println();

    printSensor(
        "FRONT",
        frontDistance
    );

    Serial.println();

    printSensor(
        "RIGHT",
        rightDistance
    );

    Serial.println();

    // -----------------------------
    // Drive forward test
    // -----------------------------

    driveForward(
        DRIVE_PWM
    );

    delay(50);

}
```

---

# How to test

## Test 1 — Lift the robot

Put the robot on a stand so wheels are free.

Upload.

Expected:

```
LEFT motor:
forward

RIGHT motor:
forward
```

If one wheel spins backwards:

Change:

```cpp
bool LEFT_REVERSED = true;
```

or:

```cpp
bool RIGHT_REVERSED = true;
```

---

## Test 2 — Put it on the floor

It should:

- move roughly straight
- not immediately spin
- not stall

If it curves:

Do **not** fix it yet.

We will measure:

```
left motor speed ≠ right motor speed
```

and later add correction.

---

## Test 3 — Serial Monitor

Set:

```
115200 baud
```

You should see:

```
----------------
LEFT: 24.5 cm
FRONT: 76.2 cm
RIGHT: 23.9 cm
```

Move a hand in front of each sensor.

Verify:

- left changes only left
- front changes only front
- right changes only right

---

## After this test, send me:

1. Does it move forward or does it rotate?
2. Which motor is reversed?
3. Serial output example:

```
LEFT:
FRONT:
RIGHT:
```

in:

- open room
- wall 10 cm in front
- wall on left side

Then V0.2 will be:

- split into `.h/.cpp` modules
- add filtered sensor class
- add proper motor calibration
- add straight-line correction.

---

If you want, I can:

- Provide complete V0.1 .ino file for straight motion test
- Add functionality to run motors backward and log sensor readings
- Add simple sensor reading averaging and fail-safe distance logging
