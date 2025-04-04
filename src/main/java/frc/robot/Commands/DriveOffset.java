package frc.robot.Commands;

import java.util.Optional;
import java.util.Set;

// Imports

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ID;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.Offsets;
import frc.robot.Subsystems.drive.Drivetrain;
import frc.robot.Subsystems.vision.Vision;
import frc.robot.Utils.CoordinateUtilities;
import frc.robot.Utils.TrapezoidController;
import org.littletonrobotics.junction.Logger;

// This Command combines DriveDistance and CenterOnTag, using Limelight and Odometry 
//      to drive to an april tag and then drive a specific offset left or right from it.
public class DriveOffset extends Command {
    // Create Drivetrain and Limelight objects
    Drivetrain dt;
    Vision vision;

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
    protected static GenericEntry yOffsetEntry = Shuffleboard.getTab("DriveOffset")
            .add("DriveOffsetYOffset", LimelightConstants.driveOffsetYOffset).getEntry();
    protected static GenericEntry omegaDps = Shuffleboard.getTab("DriveOffset")
            .add("OmegaSpeedDps", 0).getEntry();
    protected static GenericEntry angleErrorEntry = Shuffleboard.getTab("DriveOffset")
            .add("DriveOffsetAngleError", 0)
            .getEntry();
    private static GenericEntry currentLinearSpeedEntry = Shuffleboard.getTab("DriveOffset")
            .add("CurrentLinearSpeed", 0)
            .getEntry();
    private static GenericEntry decelKpEntry = Shuffleboard.getTab("DriveOffset")
            .add("DecelKp", LimelightConstants.driveOffsetKp)
            .getEntry();

    // Variables created for DriveOffset
    private boolean isLeft;
    private Optional<Integer> id = Optional.empty();
    private double xOffset;
    private double yOffset;
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
    protected double commandedSpeed;
    protected final double threshold = LimelightConstants.driveOffsetRangeMThreshold;
    private int counter;
    protected TrapezoidController trapezoidController;
    protected boolean useDashboardEntries;
    protected double constructedXOffsetM;
    protected double constructedYOffsetM;

    // Constructors
    public DriveOffset(Drivetrain dt, Vision vision, boolean isLeft) {
        this.dt = dt;
        this.vision = vision;
        this.isLeft = isLeft;
        this.useDashboardEntries = true;
        this.id = Optional.empty();
        addRequirements(dt);
        // 2D transform between robot and camera frames
        // Currently get offset from SparkJrConstants, but can change later
        cameraToRobot = new Transform2d(-1 * Offsets.cameraOffsetForwardM, 0, new Rotation2d());
    }

    public DriveOffset(Drivetrain dt, Vision vision, boolean isLeft, int id) {
        this.dt = dt;
        this.vision = vision;
        this.isLeft = isLeft;
        this.id = Optional.of(id);
        this.useDashboardEntries = true;
        addRequirements(dt);
        // 2D transform between robot and camera frames
        // Currently get offset from SparkJrConstants, but can change later
        cameraToRobot = new Transform2d(-1 * Offsets.cameraOffsetForwardM, 0, new Rotation2d());
    }

    public DriveOffset(Drivetrain dt, Vision vision, double xOffsetM, double yOffsetM) {
        this.dt = dt;
        this.vision = vision;
        this.isLeft = false;
        this.useDashboardEntries = false;
        this.constructedXOffsetM = xOffsetM;
        this.constructedYOffsetM = yOffsetM;
        addRequirements(dt);
        // 2D transform between robot and camera frames
        // Currently get offset from SparkJrConstants, but can change later
        cameraToRobot = new Transform2d(-1 * Offsets.cameraOffsetForwardM, 0, new Rotation2d());
        System.out.println("Running drive offset with set x and y offsets");
    }

    @Override
    public void initialize() {
        // Set id

        if (id.isEmpty()) {
            vision.setFiducialIDFilter(0, Constants.ID.allAprilIDs);
        } else {
            vision.setFiducialIDFilter(0, Set.of(id.get()));
            System.out.println("Set ID");
        }

        // Create a new Trapezoid profile
        if (useDashboardEntries) {
            xOffset = xOffsetEntry.getDouble(0.3);
            yOffset = yOffsetEntry.getDouble(0.0);
            // Change offset based on direction, so we can go to left or right reef goal
            if (isLeft) {
                // Left is negative Y
                yOffset *= -1;
            }
        } else {
            xOffset = constructedXOffsetM;
            yOffset = constructedYOffsetM;
        }
        // Get minimun velocity from Constants and shuffleboard
        minVel = minVelEntry.getDouble(LimelightConstants.driveOffsetMinVel);
        desiredPose = getDesiredPose();

        // Reset counter
        counter = 0;

        trapezoidController = new TrapezoidController(0.0, threshold, minVel,
                maxVelEntry.getDouble(LimelightConstants.driveOffsetMinVel),
                maxAccEntry.getDouble(LimelightConstants.driveOffsetMaxAccMSS),
                maxDccEntry.getDouble(LimelightConstants.driveOffsetMaxDccMSS),
                decelKpEntry.getDouble(1.0));
    }

    // This function uses limelight and odometry to report the desired pose relative
    // to the field
    public Pose2d getDesiredPose() {
        // Get values from limelight
        Rotation2d rotAngle = vision.getTargetYaw(0);
        double yDis = vision.getTargetDistX(0).in(Units.Meters);
        double xDis = vision.getTargetDistZ(0).in(Units.Meters);
        // Get april tag position from camera
        // TODO: this isn't what it was in the main branch because it wasn't working in
        // sim. What changed?
        Pose2d tagPose = new Pose2d(xDis, yDis,
                rotAngle.plus(Rotation2d.k180deg));
        // Calculate desired offset using shuffleboard and at 0 degrees
        Transform2d desiredOffset = new Transform2d(xOffset, yOffset, new Rotation2d());
        // Calculate robot-relative desired pose
        Pose2d desiredPoseRobotRelative = tagPose.plus(desiredOffset).plus(cameraToRobot);
        // Get current robot pose from odometry
        currentPose = dt.getRoboPose2d();
        // Get desired position from odometry
        Pose2d desiredPoseField = currentPose
                .plus(new Transform2d(desiredPoseRobotRelative.getX(), desiredPoseRobotRelative.getY(),
                        desiredPoseRobotRelative.getRotation().plus(Rotation2d.k180deg)));

        // Print outs for testing
        // System.out.println("currentPose = " + currentPose);
        // System.out.println("tagPose = " + tagPose);
        // System.out.println("desiredPoseRobotRelative = " + desiredPoseRobotRelative);
        // System.out.println("desiredPoseField = " + desiredPoseField);
        //
        Logger.recordOutput("DriveOffset/DesiredPose", desiredPoseField);

        return desiredPoseField;
    }

    @Override
    public void execute() {
        // Update desired pose every so often
        // // TODO: this works fine if we run it once, but once it runs multiple times,
        // it messes a lot up
        if (vision.isTargetValid(0) && (counter == 0)) {
            desiredPose = getDesiredPose();
        }
        counter++;
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

        double commandedVel = trapezoidController.calculate(rangeM, chassisMagnitude);

        // Calculate x and y components of desired velocity
        calcVel = CoordinateUtilities.courseSpeedToLinearVelocity(bearingDeg, commandedVel);

        // Calculate total time left
        // var currentVel = Math.max(commandedSpeed, minVel);
        double remainingTime = 1.0; // Placeholder. Calculate based on remaining distance

        // Calculate angle error
        angleError = desiredPose.getRotation().getRadians() - currentPose.getRotation().getRadians();
        if (angleError > Math.PI) {
            angleError -= 2 * Math.PI;
        }
        if (Math.abs(angleError) > LimelightConstants.driveOffsetAngleError) {
            if (remainingTime > 0) {
                // Calculate angular speed
                calcVel.omegaRadiansPerSecond = angleError / remainingTime;
                calcVel.omegaRadiansPerSecond = Math.copySign(
                        Math.max(Math.abs(calcVel.omegaRadiansPerSecond), Math.toRadians(10.0)),
                        calcVel.omegaRadiansPerSecond);
            } else {
                // Give robot a chance to turn to desired angle even after we've reached the
                // correct distance
                calcVel.omegaRadiansPerSecond = Math.toRadians(
                        Math.copySign(Math.toRadians(8.0), angleError));
            }
        }
        omegaDps.setDouble(Math.toDegrees(calcVel.omegaRadiansPerSecond));
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
    public void end(boolean interrupted) {
        // Reset values
        vision.setFiducialIDFilter(0, Constants.ID.allAprilIDs);
        System.out.println("Reset IDs");
    }

    @Override
    public boolean isFinished() {
        return rangeM <= threshold && Math.abs(angleError) <= LimelightConstants.driveOffsetAngleError;
    }
}
