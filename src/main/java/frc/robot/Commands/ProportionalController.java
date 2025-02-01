// Imports
package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Limelight;
// Depending on the robot, use different constants
//import frc.robot.TyRap24Constants.*;
import frc.robot.SparkJrConstants.*;

// This Command calculates velocities for limelight for one value
public class ProportionalController {

    // Create variables
    private double proportion;
    private double threshold;
    private double speed;
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
        // If not, meaning we still have to drive further, set the calculated velocity
        //    to be between a min and max, while using the correct sign (+ or -)
        if (Math.abs(error) < threshold) {
            desiredVel = 0;
        } else {
            desiredVel = Math.copySign(MathUtil.clamp(calculatedVel, minVel, maxVel), error);
        }

        // Commanded X Velocity ramped
        // If the desired velocity is greater than the commanded velocity (in the same direction), we accelerate
        // If not, we decelerate
        // We have to switch because there are different values for both
        if ((Math.abs(desiredVel) - Math.abs(commandedVel)) > 0) {
            deltaVel = maxAcc;
        } else {
            deltaVel = maxDcc;
        }
        // 
        if (desiredVel > commandedVel) {
            commandedVel = Math.min(desiredVel, commandedVel + deltaVel);
        } else if (desiredVel < commandedVel) {
            commandedVel = Math.max(desiredVel, commandedVel - deltaVel);
        }

        return speed = commandedVel;
    }
}
