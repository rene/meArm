/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2020 Renê de Souza Pinto
 *
 * Arduino robot arm controller with meArm joystick shield
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <math.h>
#include <Servo.h>

/** Calibrate system */
#define CALIBRATE 0

/** Left joystick: x-axis */
#define JOY_LEFT_X   A0
/** Left joystick: y-axis */
#define JOY_LEFT_Y   A1
/** Left joystick: button */
#define JOY_LEFT_BT  2
/** Right joystick: x-axis */
#define JOY_RIGHT_X  A3
/** Right joystick: y-axis */
#define JOY_RIGHT_Y  A2
/** Right joystick: button */
#define JOY_RIGHT_BT 4
/** Value when button is pressed */
#define JOY_BT_PRESSED 0
/** Value when button is released */
#define JOY_BT_RELEASED 1

/* Joystick offsets: shall be calibrated */

/** Left joystick X offset */
#define JOY_LEFT_OFFSET_X   -20
/** Left joystick Y offset */
#define JOY_LEFT_OFFSET_Y   -19
/** Right joystick X offset */
#define JOY_RIGHT_OFFSET_X  -12
/** Right joystick Y offset */
#define JOY_RIGHT_OFFSET_Y  -12

/** LED (shield) */
#define JOY_SHIELD_LED 3

/** Left servo */
#define SERVO_LEFT  11
/** Right servo */
#define SERVO_RIGHT 10
/** Base servo */
#define SERVO_BASE  9
/** Claw servo */
#define SERVO_CLAW  5

/* Angle values: must be calibrated according the arm assembly */

/** Claw servo minimum angle */
#define CLAW_MIN_ANGLE        30
/** Claw servo maximum angle */
#define CLAW_MAX_ANGLE        60
/** Claw servo initial angle */
#define CLAW_DEFAULT_ANGLE    30
/** Right servo minimum angle */
#define RIGHT_MIN_ANGLE       20
/** Right servo maximum angle */
#define RIGHT_MAX_ANGLE      170
/** Right servo initial angle */
#define RIGHT_DEFAULT_ANGLE   90
/** Left servo minimum angle */
#define LEFT_MIN_ANGLE       110
/** Left servo maximum angle */
#define LEFT_MAX_ANGLE       175
/** Left servo initial angle */
#define LEFT_DEFAULT_ANGLE   130
/** Base servo minimum angle */
#define BASE_MIN_ANGLE         5
/** Base servo maximum angle */
#define BASE_MAX_ANGLE       175
/** Base servo initial angle */
#define BASE_DEFAULT_ANGLE    90

/** Minimum joystick movement (delta) to activate the servos */
#define DELTA_LIMIT 2

/** Button holding time to save current arm position (ms) */
#define BT_TIME_SAVE     1500
/** Button holding time to restore saved arm position (ms) */
#define BT_TIME_RESTORE   200

/** Servo motors */
Servo servoLeft, servoRight, servoBase, servoClaw;

/** Left joystick read values */
int left_x, left_y;
/** Right joystick reads values */
int right_x, right_y;
/** LED state */
int led;
/** Saved position */
int savedPosition[4];

/**
 * \brief Initial setup
 */
void setup() {
  // Setup pins
  pinMode(JOY_LEFT_X, INPUT);
  pinMode(JOY_LEFT_Y, INPUT);
  pinMode(JOY_LEFT_BT, INPUT_PULLUP);

  pinMode(JOY_RIGHT_X, INPUT);
  pinMode(JOY_RIGHT_Y, INPUT);
  pinMode(JOY_RIGHT_BT, INPUT_PULLUP);

  pinMode(JOY_SHIELD_LED, OUTPUT);

  // Initialize servos
  servoLeft.attach(SERVO_LEFT);
  servoRight.attach(SERVO_RIGHT);
  servoBase.attach(SERVO_BASE);
  servoClaw.attach(SERVO_CLAW);

  // Reset position
  initialPositions();

  // Setup serial
  Serial.begin(115200);
}

/**
 * \brief Main loop
 */
void loop() {
  int i;
  int left_dx, left_dy, right_dx, right_dy;
  long pt, dt;

#if CALIBRATE == 1
  left_x  = analogRead(JOY_LEFT_X);
  left_y  = analogRead(JOY_LEFT_Y);
  right_x = analogRead(JOY_RIGHT_X);
  right_y = analogRead(JOY_RIGHT_Y);
  Serial.print("LEFT X OFFSET: "); Serial.println(left_x - 511);
  Serial.print("LEFT Y OFFSET: "); Serial.println(left_y - 511);
  Serial.print("RIGHT X OFFSET: "); Serial.println(right_x - 511);
  Serial.print("RIGHT Y OFFSET: "); Serial.println(right_y - 511);
#else
  left_x  = joystickRead(JOY_LEFT_X)  - JOY_LEFT_OFFSET_X;
  left_y  = joystickRead(JOY_LEFT_Y)  - JOY_LEFT_OFFSET_Y;
  right_x = joystickRead(JOY_RIGHT_X) - JOY_RIGHT_OFFSET_X;
  right_y = joystickRead(JOY_RIGHT_Y) - JOY_RIGHT_OFFSET_Y;
#endif

  left_dx  = map2angle(left_x);
  left_dy  = map2angle(left_y);
  right_dx = map2angle(right_x);
  right_dy = map2angle(right_y);

#if CALIBRATE == 1
  Serial.println("------------------------------");
  Serial.println("Servos:");
  Serial.print("LEFT:  "); Serial.println(servoLeft.read());
  Serial.print("RIGHT: "); Serial.println(servoRight.read());
  Serial.print("BASE:  "); Serial.println(servoBase.read());
  Serial.print("CLAW:  "); Serial.println(servoClaw.read());
#endif

  // Left servo
  if (abs(left_dy) >= DELTA_LIMIT) {
    writeServo(&servoLeft, servoLeft.read() + left_dy,
                LEFT_MIN_ANGLE, LEFT_MAX_ANGLE);
  }

  // Base
  if (abs(left_dx) >= DELTA_LIMIT) {
    writeServo(&servoBase, servoBase.read() + (left_dx * -1),
                BASE_MIN_ANGLE, BASE_MAX_ANGLE);
  }

  // Right servo
  if (abs(right_dy) >= DELTA_LIMIT) {
    writeServo(&servoRight, servoRight.read() + (right_dy * -1),
                RIGHT_MIN_ANGLE, RIGHT_MAX_ANGLE);
  }

  // Claw
  if (abs(right_dx) >= DELTA_LIMIT) {
    writeServo(&servoClaw, servoClaw.read() + right_dx,
                CLAW_MIN_ANGLE, CLAW_MAX_ANGLE);
  }

  // Check button to save or restore arm position
  if (digitalRead(JOY_RIGHT_BT) == JOY_BT_PRESSED) {
    digitalWrite(JOY_SHIELD_LED, HIGH);
    pt = millis();
    while (digitalRead(JOY_RIGHT_BT) == JOY_BT_PRESSED)
      delay(50);
    dt = millis() - pt;
    digitalWrite(JOY_SHIELD_LED, LOW);

    if (dt >= BT_TIME_SAVE) {
      // Save position
      savedPosition[0] = servoClaw.read();
      savedPosition[1] = servoRight.read();
      savedPosition[2] = servoLeft.read();
      savedPosition[3] = servoBase.read();
      blinkLED(JOY_SHIELD_LED, 3, 300);
    } else if (dt >= BT_TIME_RESTORE) {
      // Restore position
      blinkLED(JOY_SHIELD_LED, 1, 200);
      restorePosition();
    }
  }

  // Small delay for a smooth movement
  delay(15);
}

/**
 * \brief Put all servos at the initial position
 */
void initialPositions()
{
  servoClaw.write(CLAW_DEFAULT_ANGLE);
  servoRight.write(RIGHT_DEFAULT_ANGLE);
  servoLeft.write(LEFT_DEFAULT_ANGLE);
  servoBase.write(BASE_DEFAULT_ANGLE);

  savedPosition[0] = CLAW_DEFAULT_ANGLE;
  savedPosition[1] = RIGHT_DEFAULT_ANGLE;
  savedPosition[2] = LEFT_DEFAULT_ANGLE;
  savedPosition[3] = BASE_DEFAULT_ANGLE;
}

/**
 * \brief Restore saved position
 */
void restorePosition()
{
  moveTo(&servoClaw, savedPosition[0],
         CLAW_MIN_ANGLE, CLAW_MAX_ANGLE);

  moveTo(&servoRight, savedPosition[1],
         RIGHT_MIN_ANGLE, RIGHT_MAX_ANGLE);

  moveTo(&servoLeft, savedPosition[2],
         LEFT_MIN_ANGLE, LEFT_MAX_ANGLE);

  moveTo(&servoBase, savedPosition[3],
         BASE_MIN_ANGLE, BASE_MAX_ANGLE);
}

/**
 * \brief Move servos smoothly to the provided position
 * \param [out] motor Servo
 * \param [in] val Angle
 * \param [in] smin Minimum allowed angle
 * \param [in] smax Maximum allowed angle
 */
void moveTo(Servo *motor, int val, int smin, int smax)
{
  int i, pos, diff;

  if (motor != NULL) {
    pos = motor->read();
    if (pos > val) {
      for (i = pos; i > val; i--) {
        writeServo(motor, i, smin, smax);
        delay(10);
      }
    } else {
      for (i = pos; i < val; i++) {
        writeServo(motor, i, smin, smax);
        delay(10);
      }
    }
  }
}

/**
 * \brief Set servo position respecting the provided minimum and maximum allowed
 * angles
 * \param [out] motor Servo
 * \param [in] val Angle
 * \param [in] smin Minimum allowed angle
 * \param [in] smax Maximum allowed angle
 */
void writeServo(Servo *motor, int val, int smin, int smax)
{
  int cangle;
  if (motor != NULL) {
    if (val >= smin && val <= smax) {
      if (val < smin) {
        val = smin;
      } else if (val > smax) {
        val = smax;
      }
      digitalWrite(JOY_SHIELD_LED, HIGH);

      // Calculate how many degress the servo will turn
      cangle = motor->read();
      motor->write(val);

      // Wait for movement (SG-90 - 0.1s/60°)
      delay((int)(abs(val-cangle) * 1.6667));

      digitalWrite(JOY_SHIELD_LED, LOW);
    }
  }
}

/**
 * \brief Convert analog read to offset angle
 * \param [in] x Analog read [0,1023]
 * \return int Offset angle
 */
int map2angle(int x)
{
  /* Simple interpolation:
   * Line function, range: [-4,4]
   * float y = 0.0078125 * (x - 511);
   */
  /* Log function for interpolation, more smooth
   * Range: ~[-4, 4]
   */
  float y = ((4.5*logf(x + 512))/logf(2)) - 44.0;
  return (int)roundf(y);
}

/**
 * \brief Read joystick potentiometer
 * \param [in] pin Analog pin
 * \return int Analog read
 */
int joystickRead(int pin)
{
  int i, val, sum;
  sum = 0;
  for (i = 0; i < 2; i++) {
    val = analogRead(pin);
    sum += val;
  }
  // Return the average of last 2 reads
  return (sum / 2);
}

/**
 * \brief Blink LED
 * \param [in] pin Pin
 * \param [in] count Number of blinks
 * \param [in] timeout Interval time
 */
void blinkLED(int pin, int count, unsigned long timeout)
{
  int i;
  for (i = 0; i < count; i++) {
    digitalWrite(pin, HIGH);
    delay(timeout);
    digitalWrite(pin, LOW);
    delay(timeout);
  }
  digitalWrite(pin, LOW);
}
