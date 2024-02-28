#include <Arduino.h>
#include <Wire.h>
#include <SmartMotor.h>

// TEST PARAMETERS
#define DESIRED_DISTANCE_CM 50 // Desired distance in centimeters
#define INITIAL_RPM 0.0
#define MAX_RPM 65.0
#define RPM_INCREMENT 10.0
#define MOTOR_NUM 1

// MOTOR PROPERTIES
#define GEAR_RATIO 150           // MOTOR GEAR RATIO
#define ENCODER_TICKS_PER_REV 12 // NO. OF HIGH PULSES PER ROTATION
#define WHEEL_DIAMETER_CM 3.2    // Wheel diameter in centimeters
#define SHAFT_REV_TO_ENCODER_TICKS (ENCODER_TICKS_PER_REV * GEAR_RATIO)

// PID PARAMETERS
double Kp = 3.5;
double Ki = 0.005;
double Kd = 0.05;

// INIT SMART MOTORS
SmartMotor motors[MOTOR_NUM] = {0x07};
// I2C DATA PROPERTIES
#define RETURN_VAR_BYTES 5
#define RETURN_VARS 3

// MOTOR DRIVER FUNCTIONS
#define TUNE_POSITION_PID 0
#define TUNE_VELOCITY_PID 1
#define SET_POSITION 2
#define SET_VELOCITY 3
#define RESET 4

// INIT MOTORS ADDRESSES
byte motorAddrs[MOTOR_NUM] = {0x07};
// byte motorAddrs[MOTOR_NUM] = {0x04, 0x05, 0x06}; 

// INIT RETURN DATA
byte returnData[RETURN_VARS*RETURN_VAR_BYTES] {0};

struct encoded_val {
  byte coeff[4]={0};
  byte exp=0;
};

// QUERIED MOTOR VARIABLES
int32_t encoderCount;
double velocity;
int32_t motorCurrent;

bool increasing = 1;
//double targetVel = MAX_RPM;

/**
  Send motor commands to the smart motor driver.

  @param addr 8-bit motor address.
*/
int sendCmd(byte addr, byte cmd[], int cmdBytes) {
    Wire.beginTransmission(addr); // BEGIN TRANSMISSION
    Wire.write(cmd,cmdBytes); // WRITE BYTES TO THE GIVEN ADDRESS
    int status=Wire.endTransmission(); // END TRANSMISSION

    return status;
}

double decodeDouble(byte* bytes, int startIndex) {
  int32_t coeff = (((int32_t)bytes[startIndex] << 24) | ((int32_t)bytes[startIndex+1] << 16) | (((int32_t)bytes[startIndex+2] << 8) | (int32_t)bytes[startIndex+3])); // DECODE 32-BIT COEFFICIENT
  byte exp = (byte)bytes[startIndex+4]; // DECODE 8-BIT EXPONENT
  return coeff*pow(10,-exp);
}

int32_t decodeInt32(byte* bytes, int startIndex) {
  return (((int32_t)bytes[startIndex] << 24) | ((int32_t)bytes[startIndex+1] << 16) | (((int32_t)bytes[startIndex+2] << 8) | (int32_t)bytes[startIndex+3])); // DECODE 32-BIT INT
}

encoded_val encodeDouble(double val) {
  encoded_val encodedVal;

  while (fmod(val,1.0) > 0.0) {
    val *= 10;
    encodedVal.exp++;
  }

  int32_t intVal=(int32_t)val;
  encodedVal.coeff[0] = intVal >> 24;
  encodedVal.coeff[1] = intVal >> 16;
  encodedVal.coeff[2] = intVal >> 8;
  encodedVal.coeff[3] = intVal & 0xFF;

  return encodedVal;
}

encoded_val encodeInt32(int32_t val) {
  encoded_val encodedVal;

  encodedVal.coeff[0] = val >> 24;
  encodedVal.coeff[1] = val >> 16;
  encodedVal.coeff[2] = val >> 8;
  encodedVal.coeff[3] = val & 0xFF;

  return encodedVal;
}

/**
  Read query data from the smart motor driver.

  @param addr 8-bit motor address.
*/
void read(byte addr) {
    encoderCount=0;
    velocity=0.0;
    motorCurrent=0;

    Wire.requestFrom(addr,RETURN_VARS*RETURN_VAR_BYTES); // REQUEST DATA FROM ADDRESS

    // READ ALL AVAILABLE BYTES INTO A BUFFER
    int index = 0;
    while (Wire.available()) {
        byte b = Wire.read();
        returnData[index++] = b;
    }

    encoderCount=decodeInt32(returnData,0);  // DECODE ENCODER COUNT
    velocity=decodeDouble(returnData,5);  // DECODE MOTOR VELOCITY
    motorCurrent=decodeInt32(returnData,10);  //DECODE MOTOR CURRENT
}

/**
  Set target velocity.
  
  @param addr 8-bit motor address.
  @param vel Target velocity (in RPM).
*/
int setVelocity(byte addr, double vel) {
    byte cmd[6];

    // SET CMD ARRAY
    cmd[0] = SET_VELOCITY;

    encoded_val encVel=encodeDouble(vel); // ENCODE VELOCITY
    cmd[1] = encVel.coeff[0]; 
    cmd[2] = encVel.coeff[1]; 
    cmd[3] = encVel.coeff[2];
    cmd[4] = encVel.coeff[3];
    cmd[5] = encVel.exp;

    int status=sendCmd(addr,cmd,sizeof(cmd)); // SEND CMD
    return status;
}
void createTriangularVelocityProfile(byte addr) {
    // Increment velocity from INITIAL_RPM to MAX_RPM
    for (double vel = INITIAL_RPM; vel <= MAX_RPM; vel += RPM_INCREMENT) {
        setVelocity(addr, vel); // Set motor velocity
        delay(1000); // Adjust this delay as needed
        read(addr); // Read the actual velocity from the motor driver
        Serial.print(vel); // Send target velocity
        Serial.print(", ");
        Serial.println(velocity); // Send actual velocity
    }

    // Decrement velocity from MAX_RPM to INITIAL_RPM
    for (double vel = MAX_RPM; vel >= INITIAL_RPM; vel -= RPM_INCREMENT) {
        setVelocity(addr, vel); // Set motor velocity
        delay(1000); // Adjust this delay as needed
        read(addr); // Read the actual velocity from the motor driver
        Serial.print(vel); // Send target velocity
        Serial.print(", ");
        Serial.println(velocity); // Send actual velocity
    }
}
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
    // motors[i].setPosition(targetPos);

    // Set motor velocity 
    byte addr = motorAddrs[i];

    // Delay to allow the motor to reach the target position
    // delay(1000);

    // Read the actual position
    double actualPos = motors[i].getPosition();

    // Calculate the position error
    double positionError = targetPos - actualPos;

    // use velocity control while there is a certain about of error
    if (abs(positionError) > 20) {
      // going forwards 
      // if actualPos is in first half of route to target AND target is in front of actual
      if (actualPos < (0.5*targetPos) && targetPos > actualPos) { 
        for (double vel = INITIAL_RPM; vel <= MAX_RPM; vel += RPM_INCREMENT) { // increase pos RPM 
          Serial.println("INCREASING");
          setVelocity(addr, vel); // Set motor velocity
          delay(1000); // Adjust this delay as needed
        }
      // if actualPos is in second half of route to target AND target is in front of actual
      } else if (actualPos > (0.5*targetPos) && targetPos > actualPos) {
        // Decrement velocity from MAX_RPM to INITIAL_RPM
        for (double vel = MAX_RPM; vel >= INITIAL_RPM; vel -= RPM_INCREMENT) { // decrease pos RPM 
          Serial.println("DECREASING");
          setVelocity(addr, vel); // Set motor velocity
          delay(1000); // Adjust this delay as needed
        }
      }
      // going backwards 
      // if actualPos is in first half of route to target AND target is behind actual
      else if (targetPos < (0.5*actualPos) && targetPos < actualPos) { 
        for (double vel = INITIAL_RPM; vel >= -MAX_RPM; vel -= RPM_INCREMENT) { // increasingly neg RPM 
          Serial.println("INCREASING (back)");
          setVelocity(addr, vel); // Set motor velocity
          delay(1000); // Adjust this delay as needed
        }
      // if actualPos is in second half of route to target AND target is behind actual
      } else if (targetPos < (0.5*actualPos) && targetPos < actualPos) {
        // Decrement velocity from MAX_RPM to INITIAL_RPM
        for (double vel = -MAX_RPM; vel <= INITIAL_RPM; vel += RPM_INCREMENT) { // decreasingly neg RPM 
          Serial.println("DECREASING (back)");
          setVelocity(addr, vel); // Set motor velocity
          delay(1000); // Adjust this delay as needed
        }
      }
    } else {
      setVelocity(addr, 0);
      delay(1000);
    }
    
    // Send the target position, actual position, and position error to the serial port
    Serial.print(targetPos);
    Serial.print(",");
    Serial.print(actualPos);
    Serial.print(",");
    Serial.println(positionError);
  }

  delay(1000);
}
