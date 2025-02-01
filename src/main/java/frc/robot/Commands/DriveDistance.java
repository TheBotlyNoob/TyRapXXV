package frc.robot.Commands;

// Imports
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// Depending on the robot, use different constants
//import frc.robot.TyRap24Constants.*;
import frc.robot.SparkJrConstants.*;

// This Command will use the current position in odometry and desired position (theoretically by an AprilTag) 
//      to set drivetrain speeds until odometry indicates the robot is at the desired position
// This is for more precise positioning, like when we need to be close to an AprilTag
public class DriveDistance extends Command{
    // Drivetrain Object
    Drivetrain dt;
    private double currentX;
    private double currentY;
    private Rotation2d currentAngle;
    private double desiredDistance;
    private double desiredAngle;
    private double xDiff;
    private double yDiff;
    private double totalDis;
    private double calcAngle;
    private double xVel;
    private double yVel;
    private double rotVel;
    private Pose2d currentPose;

    public DriveDistance(Drivetrain dt, double desiredDistance, double desiredAngle) {
        this.dt = dt;
        this.desiredDistance = desiredDistance;
        this.desiredAngle = desiredAngle;
        addRequirements(dt);
    }

    @Override
    public void initialize() {
        // Get current position from odometry
        currentPose = dt.getRoboPose2d();
        currentX = currentPose.getX();
        currentY = currentPose.getY();
        currentAngle = dt.getGyroYawRotation2d();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
