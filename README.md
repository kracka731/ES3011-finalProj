# Position Control with Smart Motors

[This](https://github.com/mikhaildasilva/ES3011/blob/20d165d9492f7185c27da2c1a9fdc293d24631f3/Software/test/PID_tuning_test/PID_tuning_test.ino) sketch demonstrates position control of a smart motor using PID tuning. The goal is to move the motor to a specific position based on a desired distance.


## createTriangularVelocityProfile Function

The `createTriangularVelocityProfile` function creates a triangular velocity profile for a motor. It increments the motor's velocity from `INITIAL_RPM` to `MAX_RPM` and then decrements it back to `INITIAL_RPM`.
[File location](https://github.com/mikhaildasilva/ES3011/blob/df60fd108a9c9e1c1bca6b1fbec1b55bb6f93011/Software/test/velocity_control_test/velocity_control_test.ino)
### Parameters

- `addr`: The 8-bit address of the motor.

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
