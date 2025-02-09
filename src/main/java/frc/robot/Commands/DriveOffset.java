package frc.robot.Commands;

// Imports
import java.util.function.DoubleSupplier;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Limelight;
import frc.robot.Utils.CoordinateUtilities;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Utils.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
// Depending on the robot, use different constants
//import frc.robot.TyRap24Constants.*;
import frc.robot.SparkJrConstants.*;

// This Command combines DriveDistance and CenterOnTag, using Limelight and Odometry 
//      to drive to an april tag and then drive a specific offset left or right from it.
public class DriveOffset extends Command {
    // Create Drivetrain and Limelight objects
    Drivetrain dt;
    Limelight ll;

    // Shuffleboard for testing
    // Some values are taken from Constants
    private static GenericEntry rangeMEntry = Shuffleboard.getTab("DriveOffset").add("rangeM", 0).getEntry();
    private static GenericEntry bearingEntry = Shuffleboard.getTab("DriveOffset").add("bearing", 0).getEntry();
    private static GenericEntry currentXVelEntry = Shuffleboard.getTab("DriveOffset").add("currentXVel", 0)
            .getEntry();
    private static GenericEntry currentYVelEntry = Shuffleboard.getTab("DriveOffset").add("currentYVel", 0)
            .getEntry();
    private static GenericEntry minVelEntry = Shuffleboard.getTab("DriveOffset")
            .add("DriveOffsetMinVelEntry", LimelightConstants.driveOffsetMinVel).getEntry();
    private static GenericEntry maxVelEntry = Shuffleboard.getTab("DriveOffset")
            .add("DriveOffsetMaxVelEntry", LimelightConstants.driveOffsetMaxVel).getEntry();
    private static GenericEntry maxAccEntry = Shuffleboard.getTab("DriveOffset")
            .add("DriveOffsetMaxAccEntry", LimelightConstants.driveOffsetMaxAccMSS).getEntry();
    private static GenericEntry maxDccEntry = Shuffleboard.getTab("DriveOffset")
            .add("DriveOffsetMaxDccEntry", LimelightConstants.driveOffsetMaxDccMSS).getEntry();
    protected static GenericEntry xOffsetEntry = Shuffleboard.getTab("DriveOffset")
            .add("DriveOffsetXOffset", LimelightConstants.driveOffsetXOffset).getEntry();
    protected static GenericEntry yOffsetENtry = Shuffleboard.getTab("DriveOffset")
            .add("DriveOffsetYOffset", LimelightConstants.driveOffsetYOffset).getEntry();
    protected static GenericEntry angleErrorEntry = Shuffleboard.getTab("DriveOffset").add("DriveOffsetAngleError", 0)
            .getEntry();
    private static GenericEntry currentLinearSpeedEntry = Shuffleboard.getTab("DriveOffset")
            .add("CurrentLinearSpeed", 0)
            .getEntry();

    // Variables created for DriveOffset
    private boolean isLeft;
    private double xOffset;
    private double yOffset;
    private Pose2d tagPose;
    private Pose2d currentPose;
    private Pose2d desiredPose;
    protected Transform2d cameraToRobot;

    // Variables from CenterOnTag
    private double currentVel;
    private ChassisSpeeds calcVel;
    private ChassisSpeeds chassisSpeed;
    private double chassisMagnitude;
    private double minVel;
    private double rangeM;
    private double bearingDeg;
    private double angleError;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State initial;
    private TrapezoidProfile.State goal;
    protected final double threshold = LimelightConstants.driveOffsetRangeMThreshold;
    private int counter;
    protected boolean useDashboardEntries = false;
    protected DoubleSupplier distanceSupplier;

    // Constructor
    public DriveOffset(Drivetrain dt, Limelight ll, boolean isLeft) {
        this.dt = dt;
        this.ll = ll;
        this.isLeft = isLeft;
        // 2D transform between robot and camera frames
        // Currently get offset from SparkJrConstants, but can change later
        cameraToRobot = new Transform2d(-1 * Offsets.cameraOffsetForwardM, 0, new Rotation2d());
    }

    @Override
    public void initialize() {
        // Reset counter
        counter = 0;
        // Create a new Trapezoid profile
        xOffset = xOffsetEntry.getDouble(0.3);
        yOffset = yOffsetENtry.getDouble(0.0);
        // Change offset based on direction, so we can go to left or right reef goal
        if (isLeft) {
            // Left is negative Y
            yOffset *= -1;
        }
        // Get minimun velocity from Constants and shuffleboard
        minVel = minVelEntry.getDouble(LimelightConstants.driveOffsetMinVel);
        // Create a trapezoid profile
        profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(maxVelEntry.getDouble(LimelightConstants.driveOffsetMinVel),
                        maxAccEntry.getDouble(LimelightConstants.maxAccMSS)));
        desiredPose = getDesiredPose();
    }

    // This function uses limelight and odometry to report the desired pose relative to the field
    public Pose2d getDesiredPose() {
        // Get values from limelight
        double rotAngleDegrees = -1 * ll.getFilteredYawDegrees();
        double yDis = -1 * ll.getxDistanceMeters();
        double xDis = ll.getzDistanceMeters();
        // Get april tag position from camera
        tagPose = new Pose2d(xDis, yDis, new Rotation2d(Math.toRadians(rotAngleDegrees + 180.0)));
        // Calculate desired offset using shuffleboard and at 0 degrees
        Transform2d desiredOffset = new Transform2d(xOffset, yOffset, new Rotation2d());
        // Calculate robot-relative desired pose
        Pose2d desiredPoseRobotRelative = tagPose.plus(desiredOffset).plus(cameraToRobot);
        // Get current robot pose from odometry
        currentPose = dt.getRoboPose2d();
        // Get desired position from odometry
        Pose2d desiredPoseField = currentPose
                .plus(new Transform2d(desiredPoseRobotRelative.getX(), desiredPoseRobotRelative.getY(),
                        new Rotation2d(Math.toRadians(rotAngleDegrees))));
        // Print outs for testing
        System.out.println("currentPose = " + currentPose);
        System.out.println("tagPose = " + tagPose);
        System.out.println("desiredPoseRobotRelative = " + desiredPoseRobotRelative);
        System.out.println("desiredPoseField = " + desiredPoseField);

        return desiredPoseField;
    }

    @Override
    public void execute() {
        // Update desired pose every so often
        if (ll.getTimeSinceValid() == 0 && (counter % 5 == 0)) {
            desiredPose = getDesiredPose();
        }
        // Get current pose
        currentPose = dt.getRoboPose2d();
        // Calculate distance to the new position from the current one
        rangeM = CoordinateUtilities.distanceTo(currentPose, desiredPose);
        // Calculate bearing (desired angle) in degrees
        bearingDeg = CoordinateUtilities.robotBearingTo(currentPose, desiredPose);
        // Get chassis speed
        chassisSpeed = dt.getChassisSpeeds();
        // Get chassis magnitude
        chassisMagnitude = CoordinateUtilities.getChassisMagnitude(chassisSpeed);
        // Set/reset Trapezoid states for the first 20 runs
        // This is to give wheels time to turn to correct position
        // Then set current State based on current initial and goal states
        State setpoint;
        if (counter == 0 || counter > 20) {
            goal = new TrapezoidProfile.State(rangeM, 0);
            initial = new TrapezoidProfile.State(0, chassisMagnitude);
            setpoint = profile.calculate(0.02, initial, goal);
        } else {
            // Set current state based on final initial and goal states
            // For time factor, calculate based on how many times execure has run so far
            setpoint = profile.calculate(0.02 * (counter + 1), initial, goal);
        }
        // Increase counter every time execute is run (every 0.02 seconds)
        counter++;
        // Set Trapezoid profile
        var currentVel = Math.max(setpoint.velocity, minVel);
        // Calculate x and y components of desired velocity
        calcVel = CoordinateUtilities.courseSpeedToLinearVelocity(bearingDeg, currentVel);
        // Calculate total time left
        double remainingTime = profile.totalTime();
        // Calculate angle error
        angleError = desiredPose.getRotation().getRadians() - currentPose.getRotation().getRadians();
        if (remainingTime > 0) {
            // Calculate angular speed
            calcVel.omegaRadiansPerSecond = angleError / remainingTime;
        } else {
            // Give robot a chance to turn to desired angle even after we've reached the correct distance
            calcVel.omegaRadiansPerSecond = Math.toRadians(
                    Math.copySign(LimelightConstants.minAngVelocityDPS, angleError));
        }
        // Drive
        dt.driveChassisSpeeds(calcVel);
        // Set range, bearing, velocities, and angleError in Shuffleboard
        rangeMEntry.setDouble(rangeM);
        bearingEntry.setDouble(bearingDeg);
        currentXVelEntry.setDouble(calcVel.vxMetersPerSecond);
        currentYVelEntry.setDouble(calcVel.vyMetersPerSecond);
        angleErrorEntry.setDouble(Math.toDegrees(angleError));
        currentLinearSpeedEntry.setDouble(chassisMagnitude);
    }

    @Override
    public boolean isFinished() {
        if (rangeM <= threshold && Math.abs(angleError) <= LimelightConstants.driveOffsetAngleError) {
            return true;
        }
        return false;
    }
}
