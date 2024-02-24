# Control with Smart Motors

[This](https://github.com/mikhaildasilva/ES3011/blob/20d165d9492f7185c27da2c1a9fdc293d24631f3/Software/test/PID_tuning_test/PID_tuning_test.ino) sketch demonstrates position control of a smart motor using PID tuning. The goal is to move the motor to a specific position based on a desired distance.


## createTriangularVelocityProfile Function

The `createTriangularVelocityProfile` function creates a triangular velocity profile for a motor. It increments the motor's velocity from `INITIAL_RPM` to `MAX_RPM` and then decrements it back to `INITIAL_RPM`.
[File location](https://github.com/mikhaildasilva/ES3011/blob/df60fd108a9c9e1c1bca6b1fbec1b55bb6f93011/Software/test/velocity_control_test/velocity_control_test.ino)

### Algorithm

1. **Increment Phase:**
   - Increment the velocity from `INITIAL_RPM` to `MAX_RPM` in steps of `RPM_INCREMENT`.
   - For each velocity value:
     - Set the motor velocity using the `setVelocity` function.
     - Wait for a specified delay (e.g., 1000 milliseconds).
     - Read the actual velocity from the motor driver using the `read` function.
     - Print the target and actual velocities to the Serial monitor.

2. **Decrement Phase:**
   - Decrement the velocity from `MAX_RPM` to `INITIAL_RPM` in steps of `RPM_INCREMENT`.
   - For each velocity value:
     - Set the motor velocity using the `setVelocity` function.
     - Wait for a specified delay (e.g., 1000 milliseconds).
     - Read the actual velocity from the motor driver using the `read` function.
     - Print the target and actual velocities to the Serial monitor.

## Cross-verify the enconder ticks and desired distance using the formula below:

To cover a distance of 50 cm:

Number of revolutions = $$\frac{Desired Distance}{(3.2 * \pi)}$$ = 5 revolutions

Target position in encoder ticks = # of rev. * encder ticks per revolution

Target position in encoder ticks = 5 * 1800 = 9000

## PID Tuning 

   1. The PID parameters (Kp, Ki, Kd) are set using ```tunePositionPID()```. These parameters determine how the controller responds to the error between the target and actual positions.
   2. The target position is calculated based on the desired distance and the properties of the motor and wheel. This target position is then set using the ```setPosition()``` function.
   3. The motor's actual position is continuously monitored using  ```getPosition()```. The PID controller uses this feedback to adjust the motor's input and reduce the position error.
      
Using Ziegler-Nichols or any other method of your choice to tune the gains 
- Example of the values to be changed
```c
#define DESIRED_DISTANCE_CM 20 // Desired distance in centimeters
double Kp = 1.0; // Proportional gain
double Ki = 0.0; // Integral gain
double Kd = 0.0; // Derivative gain
```

