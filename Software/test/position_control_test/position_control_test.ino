#include <Arduino.h>
#include <Wire.h>

// MOTOR PROPERTIES
#define GEAR_RATIO 150            // MOTOR GEAR RATIO (110:1, 125:1, OR 150:1)
#define ENCODER_TICKS_PER_REV 12  // NO OF HIGH PULSES PER ROTATION
#define WHEEL_DIAMETER_CM 3.2     // Wheel diameter in centimeters
#define SHAFT_REV_TO_ENCODER_TICKS (ENCODER_TICKS_PER_REV * GEAR_RATIO)

// I2C DATA PROPERTIES
#define RETURN_VAR_BYTES 5
#define RETURN_VARS 3

// MOTOR DRIVER FUNCTIONS
#define SET_POSITION 2

// INIT MOTORS ADDRESS
byte motorAddr = 0x07;

// INIT RETURN DATA
byte returnData[RETURN_VARS * RETURN_VAR_BYTES] {0};

struct encoded_val {
  byte coeff[4] = {0};
  byte exp = 0;
};

// QUERIED MOTOR VARIABLES
int32_t encoderCount;
double velocity;
int32_t motorCurrent;
double desiredDistance = 50.0; //desired distance in CM
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
  Set target position.

  @param addr 8-bit motor address.
  @param pos Target position (in encoder ticks).
*/
int setPosition(byte addr, int32_t pos) {
    byte cmd[6];

    // SET CMD ARRAY
    cmd[0] = SET_POSITION;
    
    encoded_val encPos=encodeInt32(pos); // ENCODE VELOCITY
    cmd[1] = encPos.coeff[0]; 
    cmd[2] = encPos.coeff[1];
    cmd[3] = encPos.coeff[2];
    cmd[4] = encPos.coeff[3];
    cmd[5] = encPos.exp;

    int status=sendCmd(addr,cmd,sizeof(cmd)); // SEND CMD
    return status;
}

void setup() {
    Serial.begin(9600);
    Wire.begin();
}

void loop() {
    // Calculate the target position based on 20 cm distance
    double wheelCircumference = PI * WHEEL_DIAMETER_CM;
    double revolutionsRequired = desiredDistance / wheelCircumference;
    int32_t targetPos = revolutionsRequired * SHAFT_REV_TO_ENCODER_TICKS;

    // Set the motor position
    int status = setPosition(motorAddr, targetPos);
    if (status == 0) {
        // Read the actual position
        delay(1000);
        read(motorAddr);

        // Print the target and actual positions
        Serial.print("Target position: ");
        Serial.println(targetPos);
        Serial.print("Actual position: ");
        Serial.println(encoderCount);
    }

    // Stop the loop after moving 20 cm
    while (1) {
        delay(1000);
    }
}
