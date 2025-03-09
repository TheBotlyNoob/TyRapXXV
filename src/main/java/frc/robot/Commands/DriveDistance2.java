package frc.robot.Commands;

// Imports
import java.util.function.DoubleSupplier;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Utils.CoordinateUtilities;
import frc.robot.Utils.TrapezoidController;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

// This Command will use the current position in odometry to drive straight at 
// a specifie distance and direction. 
// Command will set Drivetrain speeds until odometry indicates the robot is at the desired position
// This is for more precise positioning, like when we need to be close to an AprilTag
public class DriveDistance2 extends Command {
        // Drivetrain Object
        Drivetrain dt;

        // Shuffleboard
        protected static GenericEntry desiredPosXEntry = Shuffleboard.getTab("DriveDistance2").add("desiredPoseX", 0)
                        .getEntry();
        protected static GenericEntry desiredPosYEntry = Shuffleboard.getTab("DriveDistance2").add("desiredPoseY", 0)
                        .getEntry();
        protected static GenericEntry rangeMEntry = Shuffleboard.getTab("DriveDistance2").add("rangeM", 0).getEntry();
        protected static GenericEntry bearingEntry = Shuffleboard.getTab("DriveDistance2").add("bearing", 0).getEntry();
        protected static GenericEntry currentXVelEntry = Shuffleboard.getTab("DriveDistance2").add("currentXVel", 0)
                        .getEntry();
        protected static GenericEntry currentYVelEntry = Shuffleboard.getTab("DriveDistance2").add("currentYVel", 0)
                        .getEntry();
        protected static GenericEntry timeEntry = Shuffleboard.getTab("DriveDistance2").add("time", 0).getEntry();
        protected static GenericEntry minVelEntry = Shuffleboard.getTab("DriveDistance2")
                        .add("minVelEntry", LimelightConstants.minVelocity).getEntry();
        protected static GenericEntry maxVelEntry = Shuffleboard.getTab("DriveDistance2")
                        .add("maxVelEntry", LimelightConstants.maxVelocity).getEntry();
        protected static GenericEntry maxAccEntry = Shuffleboard.getTab("DriveDistance2")
                        .add("maxAccEntry", LimelightConstants.maxAccMSS).getEntry();
        protected static GenericEntry maxDccEntry = Shuffleboard.getTab("DriveDistance2")
                        .add("maxDccEntry", LimelightConstants.maxDccMSS).getEntry();
        protected static GenericEntry decelKpEntry = Shuffleboard.getTab("DriveDistance2")
                        .add("decelKpEntry", LimelightConstants.driveDistanceProp).getEntry();
        protected static GenericEntry desiredDisEntry = Shuffleboard.getTab("DriveDistance2")
                        .add("desiredDis", 1.5).getEntry();
        protected static GenericEntry desiredAngEntry = Shuffleboard.getTab("DriveDistance2")
                        .add("desiredAng", 0).getEntry();

        // Variables
        protected double currentVel;
        protected double desiredDistance;
        protected double desiredAngle;
        protected ChassisSpeeds calcVel;
        protected ChassisSpeeds chassisSpeed;
        protected double chassisMagnitude;
        protected double minVel;
        protected double maxVel;
        protected Pose2d currentPose;
        protected Pose2d desiredPose;
        protected double rangeM;
        protected double bearingDeg;
        protected TrapezoidController profile;
        protected double threshold;
        protected boolean useDashboardEntries = false;
        protected DoubleSupplier distanceSupplier;

        // Constructors
        public DriveDistance2(Drivetrain dt, DoubleSupplier distanceSupplier, double desiredAngle) {
                this.dt = dt;
                this.distanceSupplier = distanceSupplier;
                this.desiredAngle = desiredAngle;
                addRequirements(dt);
        }

        public DriveDistance2(Drivetrain dt) {
                this.dt = dt;
                this.useDashboardEntries = true;
                addRequirements(dt);
        }

        @Override
        public void initialize() {
                try {
                        // Make sure dashboard values are used in code
                        if (this.useDashboardEntries) {
                                System.out.println("Using dashboard values");
                                this.desiredDistance = desiredDisEntry.getDouble(1.5);
                                this.desiredAngle = desiredAngEntry.getDouble(0);
                                System.out.println("dist:" + this.desiredDistance + " ang: " + this.desiredAngle);
                        } else {
                                this.desiredDistance = this.distanceSupplier.getAsDouble();
                                System.out.println("Using argument values:" + this.desiredDistance + " ang: "
                                                + this.desiredAngle);
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
                                        .plus(CoordinateUtilities.rangeAngleToTransform(this.desiredDistance,
                                                        this.desiredAngle));
                        // Shuffleboard desired pose
                        desiredPosXEntry.setValue(desiredPose.getX());
                        desiredPosYEntry.setValue(desiredPose.getY());
                        // Create a new Trapezoid profile
                        // profile = new TrapezoidProfile(
                        // new TrapezoidProfile.Constraints(maxVel,
                        // maxAccEntry.getDouble(LimelightConstants.maxAccMSS)));
                        // new TrapezoidProfile.Constraints(maxVel, maxAccEntry.getDouble(3)));

                        // Calculate proportion of current velocity in line with the desired velocity
                        chassisSpeed = dt.getChassisSpeeds();
                        var currentSpeedVec = new Vector<>(Nat.N2());
                        currentSpeedVec.set(0, 0, chassisSpeed.vxMetersPerSecond);
                        currentSpeedVec.set(1, 0, chassisSpeed.vyMetersPerSecond);

                        var offsetVector = new Vector<>(Nat.N2());
                        offsetVector.set(0, 0, desiredPose.getX());
                        offsetVector.set(1, 0, desiredPose.getY());
                        var offsetUnitVector = offsetVector.div(offsetVector.norm());
                        double speedInDesiredDirection = currentSpeedVec.dot(offsetUnitVector);

                        profile = new TrapezoidController(
                                        speedInDesiredDirection,
                                        threshold,
                                        minVel,
                                        maxVel,
                                        maxAccEntry.getDouble(LimelightConstants.driveOffsetMaxAccMSS),
                                        3,
                                        decelKpEntry.getDouble(1.0));
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
                double lastTime = Timer.getFPGATimestamp();
                // Calculate distance to the new position from the current one
                rangeM = CoordinateUtilities.distanceTo(currentPose, desiredPose);
                // Calculate bearing
                bearingDeg = CoordinateUtilities.robotBearingTo(currentPose, desiredPose);
                // Get chassis speed
                chassisSpeed = dt.getChassisSpeeds();
                // Get chassis magnitude
                chassisMagnitude = CoordinateUtilities.getChassisMagnitude(chassisSpeed);
                // Set Trapezoid profile
                var setpoint = profile.calculate(rangeM, chassisMagnitude);
                currentVel = Math.max(setpoint, minVel);
                // Calculate x and y components of desired velocity
                calcVel = CoordinateUtilities.courseSpeedToLinearVelocity(bearingDeg, currentVel);
                // Drive
                dt.driveChassisSpeeds(calcVel);
                // Set range, bearing, vel, and time in Shuffleboard
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
