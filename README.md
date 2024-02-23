## createTriangularVelocityProfile Function

The `createTriangularVelocityProfile` function creates a triangular velocity profile for a motor. It increments the motor's velocity from `INITIAL_RPM` to `MAX_RPM` and then decrements it back to `INITIAL_RPM`.
[https://github.com/mikhaildasilva/ES3011/blob/df60fd108a9c9e1c1bca6b1fbec1b55bb6f93011/Software/test/velocity_control_test/velocity_control_test.ino](File Location)
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
