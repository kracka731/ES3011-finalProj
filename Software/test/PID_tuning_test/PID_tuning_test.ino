#include <Arduino.h>
#include <Wire.h>
#include <SmartMotor.h>

// TEST PARAMETERS
#define DESIRED_DISTANCE_CM 20 // Desired distance in centimeters
#define MOTOR_NUM 1

// MOTOR PROPERTIES
#define GEAR_RATIO 150           // MOTOR GEAR RATIO
#define ENCODER_TICKS_PER_REV 12 // NO. OF HIGH PULSES PER ROTATION
#define WHEEL_DIAMETER_CM 3.2    // Wheel diameter in centimeters
#define SHAFT_REV_TO_ENCODER_TICKS (ENCODER_TICKS_PER_REV * GEAR_RATIO)

// PID PARAMETERS
double Kp = 1.0;
double Ki = 0.0;
double Kd = 0.0;

// INIT SMART MOTORS
SmartMotor motors[MOTOR_NUM] = {0x07};

void setup()
{
  Serial.begin(9600);
  Wire.begin(); // INIT DEVICE AS I2C CONTROLLER
}

void loop()
{
  // Calculate the target position based on the desired distance
  double wheelCircumference = PI * WHEEL_DIAMETER_CM;
  double revolutionsRequired = DESIRED_DISTANCE_CM / wheelCircumference;
  double targetPos = revolutionsRequired * SHAFT_REV_TO_ENCODER_TICKS;

  // FOR EACH MOTOR ADDRESS
  for (int i = 0; i < MOTOR_NUM; i++)
  {
    // Tune the PID parameters
    motors[i].tunePositionPID(Kp, Ki, Kd);

    // Set the motor position
    motors[i].setPosition(targetPos);

    // Delay to allow the motor to reach the target position
    delay(1000);

    // Read the actual position
    double actualPos = motors[i].getPosition();

    // Calculate the position error
    double positionError = targetPos - actualPos;

    // Send the target position, actual position, and position error to the serial port
    Serial.print(targetPos);
    Serial.print(",");
    Serial.print(actualPos);
    Serial.print(",");
    Serial.println(positionError);
  }

  delay(1000);
}
