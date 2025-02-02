package frc.robot.Commands;

import java.util.function.DoubleSupplier;

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
public class DriveDistance extends Command {
    // Drivetrain Object
    Drivetrain dt;

    // Shuffleboard
    private static GenericEntry desiredPosXEntry = Shuffleboard.getTab("DriveDistance").add("desiredPoseX", 0)
            .getEntry();
    private static GenericEntry desiredPosYEntry = Shuffleboard.getTab("DriveDistance").add("desiredPoseY", 0)
            .getEntry();
    private static GenericEntry rangeMEntry = Shuffleboard.getTab("DriveDistance").add("rangeM", 0).getEntry();
    private static GenericEntry bearingEntry = Shuffleboard.getTab("DriveDistance").add("bearing", 0).getEntry();
    private static GenericEntry currentXVelEntry = Shuffleboard.getTab("DriveDistance").add("currentXVel", 0)
            .getEntry();
    private static GenericEntry currentYVelEntry = Shuffleboard.getTab("DriveDistance").add("currentYVel", 0)
            .getEntry();
    private static GenericEntry timeEntry = Shuffleboard.getTab("DriveDistance").add("time", 0).getEntry();
    private static GenericEntry minVelEntry = Shuffleboard.getTab("DriveDistance")
            .add("minVelEntry", LimelightConstants.minVelocity).getEntry();
    private static GenericEntry maxVelEntry = Shuffleboard.getTab("DriveDistance")
            .add("maxVelEntry", LimelightConstants.maxVelocity).getEntry();
    private static GenericEntry maxAccEntry = Shuffleboard.getTab("DriveDistance")
            .add("maxAccEntry", LimelightConstants.maxAccMSS).getEntry();
    private static GenericEntry maxDccEntry = Shuffleboard.getTab("DriveDistance")
            .add("maxDccEntry", LimelightConstants.maxDccMSS).getEntry();
    protected static GenericEntry desiredDisEntry = Shuffleboard.getTab("DriveDistance")
            .add("desiredDis", 1.5).getEntry();
    protected static GenericEntry desiredAngEntry = Shuffleboard.getTab("DriveDistance")
            .add("desiredAng", 0).getEntry();

    // Variables
    private double currentVel;
    private double desiredDistance;
    private double desiredAngle;
    private ChassisSpeeds calcVel;
    private ChassisSpeeds chassisSpeed;
    private double chassisMagnitude;
    private double minVel;
    private double maxVel;
    private Pose2d currentPose;
    private Pose2d desiredPose;
    private double rangeM;
    private double bearingDeg;
    private Timer m_timer;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State initial;
    private TrapezoidProfile.State goal;
    private double threshold;
    protected boolean useDashboardEntries = false;
    protected DoubleSupplier distanceSupplier;

    public DriveDistance(Drivetrain dt, DoubleSupplier distanceSupplier, double desiredAngle) {
        this.dt = dt;
        this.distanceSupplier = distanceSupplier;
        this.desiredAngle = desiredAngle;
        addRequirements(dt);
    }

    public DriveDistance(Drivetrain dt) {
        this.dt = dt;
        this.useDashboardEntries = true;
        addRequirements(dt);
    }

    @Override
    public void initialize() {
        try {
            // Initialize timer
            m_timer = new Timer();

            if (this.useDashboardEntries) {
                System.out.println("Using dashboard values");
                this.desiredDistance = desiredDisEntry.getDouble(1.5);
                this.desiredAngle = desiredAngEntry.getDouble(0);
                System.out.println("dist:" + this.desiredDistance + " ang: " + this.desiredAngle);
            } else {
                this.desiredDistance = this.distanceSupplier.getAsDouble();
                System.out.println("Using argument values:" + this.desiredDistance + " ang: " + this.desiredAngle);
            }

            // Initialize threshold
            threshold = LimelightConstants.threshold;
            // Set min & max velocity
            minVel = minVelEntry.getDouble(LimelightConstants.minVelocity);
            maxVel = maxVelEntry.getDouble(LimelightConstants.maxVelocity);
            // Get current position from odometry
            currentPose = dt.getRoboPose2d();
            // Get desired position from odometry
            desiredPose = currentPose
                    .plus(CoordinateUtilities.rangeAngleToTransform(this.desiredDistance, this.desiredAngle));
            // Shuffleboard desired pose
            desiredPosXEntry.setValue(desiredPose.getX());
            desiredPosYEntry.setValue(desiredPose.getY());
            // Create a new Trapezoid profile
            profile = new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(maxVel, maxAccEntry.getDouble(LimelightConstants.maxAccMSS)));
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
        currentVel = Math.max(setpoint.velocity, minVel);

        // Calculate x and y components of desired velocity
        calcVel = CoordinateUtilities.courseSpeedToLinearVelocity(bearingDeg, currentVel);
        // Drive
        dt.driveChassisSpeeds(calcVel);
        // Shuffleboard range, bearing, vel, time
        rangeMEntry.setDouble(rangeM);
        bearingEntry.setDouble(bearingDeg);
        currentXVelEntry.setDouble(calcVel.vxMetersPerSecond);
        currentYVelEntry.setDouble(calcVel.vyMetersPerSecond);
        timeEntry.setDouble(lastTime);
    }

    @Override
    public boolean isFinished() {
        if (rangeM <= threshold) {
            return true;
        }
        return false;
    }
}
