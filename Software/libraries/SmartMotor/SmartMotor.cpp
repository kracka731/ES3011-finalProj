#include "SmartMotor.h"

SmartMotor::SmartMotor(byte addr) : _addr(addr) {}

int SmartMotor::tunePositionPID(double kp, double ki, double kd) {
    return SmartMotor::_tunePID(TUNE_POSITION_PID,kp,ki,kd);
}

int SmartMotor::tuneVelocityPID(double kp, double ki, double kd) {
    return SmartMotor::_tunePID(TUNE_VELOCITY_PID,kp,ki,kd);
}

int SmartMotor::setPosition(int32_t pos) {
    SmartMotor::EncodedNum encPos=SmartMotor::_encodeInt32(pos); // ENCODE VELOCITY
    return SmartMotor::_setPIDTarget(SET_POSITION,encPos); // SEND CMD
}

int SmartMotor::setVelocity(double vel) {
    SmartMotor::EncodedNum encVel=SmartMotor::_encodeInt32(vel); // ENCODE VELOCITY
    return SmartMotor::_setPIDTarget(SET_VELOCITY,encVel); // SEND CMD
}

int SmartMotor::reset() {
    byte cmd[1];

    // SET CMD ARRAY
    cmd[0] = RESET;

    return SmartMotor::_sendCmd(cmd,sizeof(cmd)); // SEND CMD
}

int32_t SmartMotor::getPosition() {
    SmartMotor::ReturnData rtrnData=SmartMotor::_readData();
    return rtrnData.encoderCount;
}

double SmartMotor::getVelocity() {
    SmartMotor::ReturnData rtrnData=SmartMotor::_readData();
    return rtrnData.velocity;
}

byte SmartMotor::getAddress() {
    return _addr;
}

int32_t SmartMotor::getMotorCurrent() {
    SmartMotor::ReturnData rtrnData=SmartMotor::_readData();
    return rtrnData.motorCurrent;
}

int SmartMotor::_sendCmd(byte* cmd, int cmdBytes) {
    Wire.beginTransmission(_addr); // BEGIN TRANSMISSION
    Wire.write(cmd,cmdBytes); // WRITE BYTES TO THE MOTOR ADDRESS
    int status=Wire.endTransmission(); // END TRANSMISSION

    return status;
}

double SmartMotor::_decodeDouble(byte* bytes, int startIndex) {
    int32_t coeff = (((int32_t)bytes[startIndex] << 24) | ((int32_t)bytes[startIndex+1] << 16) | (((int32_t)bytes[startIndex+2] << 8) | (int32_t)bytes[startIndex+3])); // DECODE 32-BIT COEFFICIENT
    byte exp = (byte)bytes[startIndex+4]; // DECODE 8-BIT EXPONENT
    return coeff*pow(10,-exp);
}

int32_t SmartMotor::_decodeInt32(byte* bytes, int startIndex) {
    return (((int32_t)bytes[startIndex] << 24) | ((int32_t)bytes[startIndex+1] << 16) | (((int32_t)bytes[startIndex+2] << 8) | (int32_t)bytes[startIndex+3])); // DECODE 32-BIT INT
}

SmartMotor::EncodedNum SmartMotor::_encodeDouble(double val) {
    SmartMotor::EncodedNum encData;

    while (fmod(val,1.0) > 0.0) {
    val *= 10;
    encData.exp++;
    }

    int32_t intVal=(int32_t)val;
    encData.coeff[0] = intVal >> 24;
    encData.coeff[1] = intVal >> 16;
    encData.coeff[2] = intVal >> 8;
    encData.coeff[3] = intVal & 0xFF;

    return encData;
}

SmartMotor::EncodedNum SmartMotor::_encodeInt32(int32_t val) {
    SmartMotor::EncodedNum encData;

    encData.coeff[0] = val >> 24;
    encData.coeff[1] = val >> 16;
    encData.coeff[2] = val >> 8;
    encData.coeff[3] = val & 0xFF;

    return encData;
}

SmartMotor::ReturnData SmartMotor::_readData() {
    Wire.requestFrom(_addr,RETURN_VARS*RETURN_VAR_BYTES); // REQUEST DATA FROM ADDRESS

    // READ ALL AVAILABLE BYTES INTO A BUFFER
    byte data[RETURN_VARS*RETURN_VAR_BYTES];
    int index = 0;
    while (Wire.available()) {
        byte b = Wire.read();
        data[index++] = b;
    }

    SmartMotor::ReturnData rtrnData;
    rtrnData.encoderCount=SmartMotor::_decodeInt32(data,0);  // DECODE ENCODER COUNT
    rtrnData.velocity=SmartMotor::_decodeDouble(data,5);  // DECODE MOTOR VELOCITY
    rtrnData.motorCurrent=SmartMotor::_decodeInt32(data,10);  //DECODE MOTOR CURRENT
    
    return rtrnData;
}

int SmartMotor::_tunePID(byte func, double kp, double ki, double kd) {
    byte cmd[16];

    // SET CMD ARRAY
    cmd[0] = func;

    double K[]={kp,ki,kd};

    for (int i=0; i<3; i++) {
      SmartMotor::EncodedNum encK=SmartMotor::_encodeDouble(K[i]); // ENCODE VELOCITY
      cmd[5*i+1] = encK.coeff[0]; 
      cmd[5*i+2] = encK.coeff[1];
      cmd[5*i+3] = encK.coeff[2];
      cmd[5*i+4] = encK.coeff[3];
      cmd[5*i+5] = encK.exp;
    }

    return SmartMotor::_sendCmd(cmd,sizeof(cmd)); // SEND CMD
}

int SmartMotor::_setPIDTarget(byte funcIndex, SmartMotor::EncodedNum encData) {
    byte cmd[6];

    // SET CMD ARRAY
    cmd[0] = funcIndex;

    cmd[1] = encData.coeff[0]; 
    cmd[2] = encData.coeff[1];
    cmd[3] = encData.coeff[2];
    cmd[4] = encData.coeff[3];
    cmd[5] = encData.exp;

    return SmartMotor::_sendCmd(cmd,sizeof(cmd)); // SEND CMD

}