package frc.robot.Commands;

// Imports
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.DriveDistance;
import frc.robot.Subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Utils.CoordinateUtilities;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Utils.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
// Depending on the robot, use different constants
//import frc.robot.TyRap24Constants.*;
import frc.robot.SparkJrConstants.*;

// This Command uses DriveDistance Command and the range sensor to drive until the range is a certain value or has gone too far
public class DriveRange extends DriveDistance {
    private double rangeThreshold;
    private DoubleSupplier rangeSupplier;
    
    // Constructors invoking DriveDistance
    public DriveRange(Drivetrain dt, DoubleSupplier maxDistanceSupplier, DoubleSupplier rangeSupplier, double desiredAngle, double rangeThreshold){
        super(dt, maxDistanceSupplier, desiredAngle);
        this.rangeSupplier = rangeSupplier;
        this.rangeThreshold = rangeThreshold;
    }

    public DriveRange(Drivetrain dt) {
        super(dt);
    }
    
    @Override
    public boolean isFinished() {
        System.out.println("Range val: "+rangeSupplier.getAsDouble());
        if (rangeSupplier.getAsDouble() >= rangeThreshold || super.isFinished()) {
            return true;
        }
        return false;
    }
}
