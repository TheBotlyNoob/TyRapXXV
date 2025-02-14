# Important Info

## CANCoder
### Replacing Them
If you replace one, you will HAVE to update a set of things in the code. We store a checklist here, so we can remember to change stuff when this happens.
1. Make all drive motors and encoders consistent i.e. all 4 should be positive when spinning in the same direction e.g. forward
    1. Turn the wheels so they are facing forward from the perspective of the robot.
    2. When you turn the wheels controlled by the drive motor, you want all the drive motors to have positive values when spun in one direction e.g. forward and then you want all the drive motors to have negative values when spun in the opposite direction e.g. backward.
        * Overload the disabledPeriodic in Robot to send Swerve Module data to SmartDashboard. This will allow you to gather the needed constants without enabling the robot.
2. Update the Encoder offset
    * Overload the disabledPeriodic to send Swerve Module data to SmartDashboard to update this offset
    1. Use a long, straight edge to make sure that all the swerve modules are facing the same direction.
    2. Take the values from Smart Dashboard (which should include the CANCoder readings) and plug them into the encoder offsets in Constants. Make sure that you are using the correct units (radians) and that the values are in the correct range (-PI to PI).
3. Make sure the **Turning Encoder** direction matches the **Turning Motor** direction
    * If you notice that the swerve module oscillates irregularly especially with no input and if it's not chasing the target value, this could be the cause i.e. that the encoder and motor disagree about which direction is positive.
4. Re-do tuning (see below)
## Tuning
### Feed forward and PID
We have found success in combining feed forward and PID. This [video](https://youtu.be/FW_ay7K4jPE?si=0itqxVwz7ds_SQRs) does a great job giving an intro comparison of feed forward and PID. You can also view the documentation through WPILib. Below, we describe our current understanding.

### Motor controls
We use SPARK Maxes to control the motors. The SPARK Max has two high level set functions to make them work. The one we use is set voltage which allows you to specify how much voltage to direct to a given motor.

The tuning constants below start at 0. We then recommend tuning the feed forward values and then the PID values. We break down the tuning of these two parts and their associated constants below.

#### Feed forward (voltage)
Tuning feed forward consists of tuning a set of constants: ks, kv, and ka.
* ks
    * The static component can be interpreted to mean the static, minimum amount of voltage you need to supply to cause the motor to turn.
    * Tuning: Increase the value until the motor moves then decrease it a little.
* kv
    * The velocity component can be interpreted to mean the amount of voltage you need to supply to maintain a given velocity.
    * Tuning: Plot the actual speed the motor is going and the target speed the motor should be going. Tune the kv value until the actual speed matches the target speed. We'll use PID tuning to decrease how long it takes for the actual speed to match the target speed.
* ka
    * We don't usually use this value.
    * The acceleration component can be interpreted to mean the amount of voltage you need to supply to maintain an acceleration.
    * Tuning: We haven't tuned this before but it likely proceeds similar to kv with plotting the actual and target acceleration and tuning the value until the actual acceleration matches the target.

#### PID (voltage)
Tuning PID occurs after tuning Feed Forward to account for the error in your feed forward model, decreasing the amount of time needed to achieve the target speed without significant under or over shooting, and to account for the remaining sources of error encountered that feed forward can't account for.

There are three constants with PID: p, i, and d.
* p
    * The Proportional term attempts to drive the position error to zero by contributing to the control signal proportionally to the current position error. (WPILib docs)
    * Tuning: Increase until the motor occilates consistently (the same amplitude back and forth over time). Halve the term. This is your starting point. Continue to tune as high as possible without occilation.
* d
    * The Derivative term attempts to drive the derivative of the error to zero by contributing to the control signal proportionally to the derivative of the error. (WPILib docs)
    * Tuning: This is a proportional term for velocity. We don't have a good strategy for tuning this other than increasing slowly and measuring performance (quickly getting to the desired speed and staying there). Feel free to contribute a good algorithm.
* i
    * The Integral term attempts to drive the total accumulated error to zero by contributing to the control signal proportionally to the sum of all past errors. (WPILib docs)
    * The WPILib docs mention using feed forward instead which we do, so we haven't needed to tune this value.
