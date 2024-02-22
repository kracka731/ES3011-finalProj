#ifndef SmartMotor_h
  #define SmartMotor_h
  #define LIBRARY_VERSION	1.0.0

  #include <Arduino.h>
  #include <Wire.h>

  class SmartMotor {
    public:
        /**
         * Sets the motor address
         */
        SmartMotor(byte);

        /**
         * Sets the position PID gains
         */
        int tunePositionPID(double, double, double);

        /**
         * Sets the velocity PID gains
         */
        int tuneVelocityPID(double, double, double);
        
        /**
         * Sets the target position
         */
        int setPosition(int32_t);
                
        /**
         * Sets the target velocity
         */
        int setVelocity(double);
                
        /**
         * Resets the motor position
         */
        int reset();
        
        /**
         * Gets the current position
         */
        int32_t getPosition();
                
        /**
         * Gets the current velocity
         */
        double getVelocity();

        /**
         * Gets the motor current
         */
        int32_t getMotorCurrent();

        /**
         * Gets the motor address
         */
        byte getAddress();

    private:
        // I2C DATA PROPERTIES
        #define RETURN_VAR_BYTES 5
        #define RETURN_VARS 3

        // MOTOR DRIVER FUNCTIONS
        #define TUNE_POSITION_PID 0
        #define TUNE_VELOCITY_PID 1
        #define SET_POSITION 2
        #define SET_VELOCITY 3
        #define RESET 4

        byte _addr; // MOTOR ADDRESS

        struct EncodedNum {
            byte coeff[4]={0};
            byte exp=0;
        };

        struct ReturnData {
            int32_t encoderCount;
            double velocity;
            int32_t motorCurrent;
        };

        /**
         * Sends smart motor command to motor address
        */
        int _sendCmd(byte*, int);

        /**
         * Decodes the encoded number and returns a double value
        */
        double _decodeDouble(byte*, int);

        /**
         * Decodes the encoded number and returns an 32-bit integer value
        */
        int32_t _decodeInt32(byte*, int);

        /**
         * Encodes the given double value
        */
        EncodedNum _encodeDouble(double);

        /**
         * Encodes the given 32-bit integer value
        */
        EncodedNum _encodeInt32(int32_t);

        /**
         * Reads query data from the smart motor driver.
        */
        ReturnData _readData();

        /**
         * Sets the gains for the position or velocity PID
        */
        int _tunePID(byte, double, double, double);
        
        /**
         * Sets the target for the position or velocity PID
        */
        int _setPIDTarget(byte funcIndex, EncodedNum encData);
  };
#endif