// Imports
package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;

// This Command calculates velocities for limelight for one value
public class ProportionalController {

    // Create variables
    private double proportion;
    private double threshold;
    private double desired;
    private double error;
    private double minVel;
    private double maxVel;
    private double maxAcc;
    private double maxDcc;
    private double commandedVel;

    // Constructor
    public ProportionalController(double proportion, double threshold, double desired, double minVel, double maxVel, double maxAcc, double maxDcc) {
        this.proportion = proportion;
        this.threshold = threshold;
        this.desired = desired;
        this.minVel = minVel;
        this.maxVel = maxVel;
        this.maxAcc = maxAcc;
        this.maxDcc = maxDcc;
    }

    // Start Calculations
    public double calculate(double current) {
        error = current - desired;
        double calculatedVel = Math.abs(error * proportion);
        double desiredVel;
        double deltaVel;

        // Desired Velocities
        // If the error is less than the threshold, meaning we have driven to the correct place, 
        //    stop the robot by setting speed to 0
        if (Math.abs(error) < threshold) {
            desiredVel = 0;
        // If not, meaning we still have to drive further, set the calculated velocity
        //    to be between a min and max, while using the correct sign (+ or -)
        } else {
            desiredVel = Math.copySign(MathUtil.clamp(calculatedVel, minVel, maxVel), error);
        }

        // Commanded X Velocity ramped
        // If the desired velocity is greater than the commanded velocity (in the same direction), we accelerate
        if ((Math.abs(desiredVel) - Math.abs(commandedVel)) > 0) {
            deltaVel = maxAcc;
        // If not, we decelerate
        // We have to switch because there are different values for both
        } else {
            deltaVel = maxDcc;
        }
        // Set commanded velocity based on acceleration or deceleration
        // Acceleration
        if (desiredVel > commandedVel) {
            commandedVel = Math.min(desiredVel, commandedVel + deltaVel);
        // Deceleration
        } else if (desiredVel < commandedVel) {
            commandedVel = Math.max(desiredVel, commandedVel - deltaVel);
        }

        // Return final velocity
        return commandedVel;
    }
}
