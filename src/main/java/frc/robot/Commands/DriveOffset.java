package frc.robot.Commands;

// Imports
import java.util.function.DoubleSupplier;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Limelight;
import frc.robot.Utils.CoordinateUtilities;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Utils.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

// This Command combines DriveDistance and CenterOnTag, using Limelight to drive to an april tag and then drive a specific offset left or right from it
public class DriveOffset extends Command {
    // Drivetrain and Limelight objects
    Drivetrain dt;
    Limelight ll;

    // Shuffleboard
    private static GenericEntry rangeMEntry = Shuffleboard.getTab("DriveOffset").add("rangeM", 0).getEntry();
    private static GenericEntry bearingEntry = Shuffleboard.getTab("DriveOffset").add("bearing", 0).getEntry();
    private static GenericEntry currentXVelEntry = Shuffleboard.getTab("DriveOffset").add("currentXVel", 0)
            .getEntry();
    private static GenericEntry currentYVelEntry = Shuffleboard.getTab("DriveOffset").add("currentYVel", 0)
            .getEntry();
    private static GenericEntry minVelEntry = Shuffleboard.getTab("DriveDistance")
            .add("minVelEntry", LimelightConstants.minVelocity).getEntry();
    private static GenericEntry maxVelEntry = Shuffleboard.getTab("DriveOffset")
            .add("maxVelEntry", LimelightConstants.maxVelocity).getEntry();
    private static GenericEntry maxAccEntry = Shuffleboard.getTab("DriveOffset")
            .add("maxAccEntry", LimelightConstants.maxAccMSS).getEntry();
    private static GenericEntry maxDccEntry = Shuffleboard.getTab("DriveOffset")
            .add("maxDccEntry", LimelightConstants.maxDccMSS).getEntry();
    protected static GenericEntry xOffsetEntry = Shuffleboard.getTab("DriveOffset")
        .add("xOffset", 0.4).getEntry();
    protected static GenericEntry yOffsetENtry = Shuffleboard.getTab("DriveOffset")
        .add("yOffset", 0.3).getEntry();

    // Variables
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
    private TrapezoidProfile profile;
    private TrapezoidProfile.State initial;
    private TrapezoidProfile.State goal;
    protected final double threshold = 0.02;
    private int counter;
    protected boolean useDashboardEntries = false;
    protected DoubleSupplier distanceSupplier;

    // Constructor
    public DriveOffset(Drivetrain dt, Limelight ll, boolean isLeft) {
        this.dt = dt;
        this.ll = ll;
        this.isLeft = isLeft;
        // 2D transform between robot and camera frames
        cameraToRobot = new Transform2d(-1*LimelightConstants.cameraOffsetForwardM, 0, new Rotation2d());
    }

    @Override
    public void initialize() {
        // Create a new Trapezoid profile
        xOffset = xOffsetEntry.getDouble(0.3);
        yOffset = yOffsetENtry.getDouble(0.0);
        if (!isLeft) {
                // Right is negative Y
                yOffset *= -1;
        }
        minVel = minVelEntry.getDouble(0.1);
        profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(maxVelEntry.getDouble(1.5), 
                maxAccEntry.getDouble(LimelightConstants.maxAccMSS)));
        desiredPose = getDesiredPose();
    }

    public Pose2d getDesiredPose() {
        // Get values from limelight
        double rotAngleDegrees = -1 * ll.getFilteredYawDegrees();
        double yDis = -1 * ll.getxDistanceMeters();
        double xDis = ll.getzDistanceMeters();
        // Get tag position from camera
        tagPose = new Pose2d(xDis, yDis, new Rotation2d(Math.toRadians(rotAngleDegrees+180.0)));

        Transform2d desiredOffset = new Transform2d(xOffset, yOffset, new Rotation2d());

        Pose2d desiredPoseRobotRelative = tagPose.plus(desiredOffset).plus(cameraToRobot);

        // Get current robot pose
        currentPose = dt.getRoboPose2d();

        // Get desired position from odometry
        Pose2d desiredPoseField = currentPose
                .plus(new Transform2d(desiredPoseRobotRelative.getX(), desiredPoseRobotRelative.getY(),
                        new Rotation2d()));
        
        System.out.println("currentPose = " + currentPose);
        System.out.println("tagPose = " + tagPose);
        System.out.println("desiredPoseRobotRelative = "+desiredPoseRobotRelative);
        System.out.println("desiredPoseField = "+desiredPoseField);
        
        return desiredPoseField;
    }

    @Override
    public void execute() {
        // Update desired pose every so often
        if (ll.getTimeSinceValid() == 0 && (counter%5==0)) {
            desiredPose = getDesiredPose();
        }
        counter++;
        // Get current pose
        currentPose = dt.getRoboPose2d();
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
        // Set range, bearing, vel, and time in Shuffleboard
        rangeMEntry.setDouble(rangeM);
        bearingEntry.setDouble(bearingDeg);
        currentXVelEntry.setDouble(calcVel.vxMetersPerSecond);
        currentYVelEntry.setDouble(calcVel.vyMetersPerSecond);
    }
    
    @Override
    public boolean isFinished() {
        if (rangeM <= threshold) {
                return true;
        }
        return false;
    }
}
