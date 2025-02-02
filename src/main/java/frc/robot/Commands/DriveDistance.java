package frc.robot.Commands;

// Imports
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
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

// This Command will use the current position in odometry and desired position (theoretically by an AprilTag) 
//      to set drivetrain speeds until odometry indicates the robot is at the desired position
// This is for more precise positioning, like when we need to be close to an AprilTag
public class DriveDistance extends Command{
    // Drivetrain Object
    Drivetrain dt;

    // Shuffleboard
    private static GenericEntry desiredPosXEntry = Shuffleboard.getTab("Limelight").add("desiredPoseX", 0).getEntry();
    private static GenericEntry desiredPosYEntry = Shuffleboard.getTab("Limelight").add("desiredPoseY", 0).getEntry();
    private static GenericEntry rangeMEntry = Shuffleboard.getTab("Limelight").add("rangeM", 0).getEntry();
    private static GenericEntry bearingEntry = Shuffleboard.getTab("Limelight").add("bearing", 0).getEntry();
    private static GenericEntry currentXVelEntry = Shuffleboard.getTab("Limelight").add("currentXVel", 0).getEntry();
    private static GenericEntry currentYVelEntry = Shuffleboard.getTab("Limelight").add("currentYVel", 0).getEntry();
    private static GenericEntry timeEntry = Shuffleboard.getTab("Limelight").add("time", 0).getEntry();

    // Variables
    private double currentX;
    private double currentY;
    private double currentVel;
    private Rotation2d currentAngle;
    private double desiredDistance;
    private double desiredX;
    private double desiredY;
    private double desiredAngle;
    private double xDiff;
    private double yDiff;
    private double totalDis;
    private double calcAngle;
    private ChassisSpeeds calcVel;
    private ChassisSpeeds chassisSpeed;
    private double chassisMagnitude;
    private double xVel;
    private double yVel;
    private double desiredVel;
    private double rotVel;
    private Pose2d currentPose;
    private Pose2d desiredPose;
    private double rangeM;
    private double bearingDeg;
    private Pose2d offset;
    private Timer m_timer;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State initial;
    private TrapezoidProfile.State goal;
    private ProportionalController controller;
    private double proportion;
    private double threshold;

    public DriveDistance(Drivetrain dt, double desiredDistance, double desiredAngle) {
        this.dt = dt;
        this.desiredDistance = desiredDistance;
        this.desiredAngle = desiredAngle;
        addRequirements(dt);
    }

    @Override
    public void initialize() {
        try {
            // Initialize timer
            m_timer = new Timer();
            // Initialize proportion
            proportion = LimelightConstants.proportion;
            // Initialize threshold
            threshold = LimelightConstants.threshold;
            // Get current position from odometry
            currentPose = dt.getRoboPose2d();
            // Get desired position from odometry
            desiredPose = currentPose.plus(CoordinateUtilities.rangeAngleToTransform(desiredDistance, desiredAngle));
            // Shuffleboard desired pose
            desiredPosXEntry.setValue(desiredPose.getX());
            desiredPosYEntry.setValue(desiredPose.getY());
            // Create a new Trapezoid profile
            profile = new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(LimelightConstants.maxVelocity, LimelightConstants.maxAccMSS));
        } catch (Exception e) {
            System.out.println("Exception initializing DriveDistance");
            e.printStackTrace();
        }

    }

    @Override
    public void execute() {
        // Get current pose
        currentPose = dt.getRoboPose2d();
        // Get current time
        double lastTime = m_timer.getFPGATimestamp();
        // Calculate distance to the new position from the current one
        rangeM = CoordinateUtilities.distanceTo(currentPose, desiredPose);
        // Calculate bearing
        bearingDeg = CoordinateUtilities.robotBearingTo(currentPose, desiredPose);
        // Get chassis speed
        chassisSpeed = dt.getChassisSpeeds();
        // Get chassis magnitude
        chassisMagnitude = CoordinateUtilities.getChassisMagnitude(chassisSpeed);
        // Set Trapezoid states
        initial = new TrapezoidProfile.State(0, chassisMagnitude);
        goal = new TrapezoidProfile.State(rangeM, 0);
        // Set Trapezoid profile
        var setpoint = profile.calculate(0.02, initial, goal);
        currentVel = setpoint.velocity;
        // Calculate x and y components of desired velocity
        calcVel = CoordinateUtilities.courseSpeedToLinearVelocity(bearingDeg, currentVel);
        // Drive
        dt.drive(calcVel.vxMetersPerSecond, calcVel.vyMetersPerSecond, calcVel.omegaRadiansPerSecond);
        // Shuffleboard range, bearing, vel, time
        rangeMEntry.setDouble(rangeM);
        bearingEntry.setDouble(bearingDeg);
        currentXVelEntry.setDouble(calcVel.vxMetersPerSecond);
        currentYVelEntry.setDouble(calcVel.vyMetersPerSecond);
        timeEntry.setDouble(lastTime);
    }

    @Override
    public boolean isFinished() {
        if(rangeM<=threshold){
            return true;
        }
        return false;
    }
}
