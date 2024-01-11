// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ID;
import frc.robot.Constants.Offsets;
import frc.robot.Utils.LimelightHelpers;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveTrain;
import frc.robot.Constants.gamePieceIDs;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
    public static enum DirectionOption {
        FORWARD,
        BACKWARD
    }
    public static final double kMaxPossibleSpeed = 5; // meters per second
    public static final double kMaxAngularSpeed = 3 * Math.PI; // per second

    private final Translation2d m_frontLeftLocation = new Translation2d(
            DriveTrain.kDistanceMiddleToFrontMotor * DriveTrain.kXForward,
            DriveTrain.kDistanceMiddleToSideMotor * DriveTrain.kYLeft);
    private final Translation2d m_frontRightLocation = new Translation2d(
            DriveTrain.kDistanceMiddleToFrontMotor * DriveTrain.kXForward,
            DriveTrain.kDistanceMiddleToSideMotor * DriveTrain.kYRight);
    private final Translation2d m_backLeftLocation = new Translation2d(
            DriveTrain.kDistanceMiddleToFrontMotor * DriveTrain.kXBackward,
            DriveTrain.kDistanceMiddleToSideMotor * DriveTrain.kYLeft);
    private final Translation2d m_backRightLocation = new Translation2d(
            DriveTrain.kDistanceMiddleToFrontMotor * DriveTrain.kXBackward,
            DriveTrain.kDistanceMiddleToSideMotor * DriveTrain.kYRight);

    private final SwerveModule m_frontLeft = new SwerveModule("FrontLeft",
            ID.kFrontLeftDrive,
            ID.kFrontLeftTurn,
            ID.kFrontLeftCANCoder,
            Offsets.kFrontLeftOffset,
            DriveTrain.turnPID,
            DriveTrain.drivePID,
            DriveTrain.turnFeedForward,
            DriveTrain.driveFeedForward);
    private final SwerveModule m_frontRight = new SwerveModule("FrontRight",
            ID.kFrontRightDrive,
            ID.kFrontRightTurn,
            ID.kFrontRightCANCoder,
            Offsets.kFrontRightOffset,
            DriveTrain.turnPID,
            DriveTrain.drivePID,
            DriveTrain.turnFeedForward,
            DriveTrain.driveFeedForward);
    private final SwerveModule m_backLeft = new SwerveModule("BackLeft",
            ID.kBackLeftDrive,
            ID.kBackLeftTurn,
            ID.kBackLeftCANCoder,
            Offsets.kBackLeftOffset,
            DriveTrain.turnPID,
            DriveTrain.drivePID,
            DriveTrain.turnFeedForward,
            DriveTrain.driveFeedForward);
    private final SwerveModule m_backRight = new SwerveModule("BackRight",
            ID.kBackRightDrive,
            ID.kBackRightTurn,
            ID.kBackRightCANCoder,
            Offsets.kBackRightOffset,
            DriveTrain.turnPID,
            DriveTrain.drivePID,
            DriveTrain.turnFeedForward,
            DriveTrain.driveFeedForward);

    private final Pigeon2 m_gyro;

    private boolean fieldRelative = true;
    private DirectionOption m_forwardDirection = DirectionOption.FORWARD;
    private final ShuffleboardTab m_driveTab = Shuffleboard.getTab("drive subsystem");
    private final SimpleWidget m_fieldRelativeWidget = m_driveTab.add("drive field relative", fieldRelative);

    /**
     * The order that you initialize these is important! Later uses of functions
     * like toSwerveModuleStates will return the same order that these are provided.
     * See
     * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
     */
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveDriveOdometry m_odometry;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
    private Pose2d robotFieldPosition;
    private boolean lockTargetInAuto = false;
    private SimpleMotorFeedforward aimFF = new SimpleMotorFeedforward(0.0, 8);
    private ProfiledPIDController aimPID = new ProfiledPIDController(0.05, 0, 0.0, new Constraints(Math.toRadians(180), Math.toRadians(180)));

    public Drivetrain(Pigeon2 m_gyro) {
        // Zero at beginning of match. Zero = whatever direction the robot (more specifically the gyro) is facing
        this.m_gyro = m_gyro;
        this.resetGyro();

        m_odometry = new SwerveDriveOdometry(
            m_kinematics,
            getGyroYawRotation2d(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            },
            new Pose2d(
                new Translation2d(2.0, 6.0), Rotation2d.fromDegrees(0.0)));
        
        robotFieldPosition = getRoboPose2d();

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                this::getRoboPose2d, // Robot pose supplier
                this::resetOdo, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveInAuto, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(7.4, 0.0, 0.0), // Translation PID constants p used to be 7
                        new PIDConstants(5.4, 0.0, 0.0), // Rotation PID constants
                        kMaxPossibleSpeed, // Max module speed, in m/s
                        DriveTrain.kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }


    public double getAngleToSpeaker() {
        double toSpeakerAngle = Math.toDegrees(Math.atan((5.55 - getRoboPose2d().getY())/(getRoboPose2d().getX() - 0.3)));
    // if(!usingVision)
        return toSpeakerAngle;
    // else
        // return getRobotRotToTarg();
    }

    /**
     * @return radians
     */
    public double getSpeakerAimTargetAngle() { //when geting apriltag data, must invert dir with negative sign to match swerve (try this on tues)
        double Vy = getChassisSpeeds().vyMetersPerSecond + 5*Math.sin(hasTarget() ? -Math.toRadians(getFRAngleToTarget()) : Math.toRadians(getAngleToSpeaker()));
        double Vx = getChassisSpeeds().vxMetersPerSecond + 5*Math.cos(hasTarget() ? -Math.toRadians(getFRAngleToTarget()) : Math.toRadians(getAngleToSpeaker()));
        return 2*Math.toRadians(getAngleToSpeaker()) - Math.atan(Vy/Vx);
    }

    private int targetID = 1;

    public void setTarget(int targetID) {
        this.targetID = targetID;
    }

    public int getTarget() {
        return this.targetID;
    }

    public boolean hasTarget() {
        return LimelightHelpers.getFiducialID("") == getTarget();
    }

    /**
     * @return degrees to the target, right is +, left is -
     */
    public double getAngleToTarget() {
        if(hasTarget()) {
        return LimelightHelpers.getTX("");
        }
        else return 0;
    }

    /**
     * @return degrees to the target, right is -, left is +
     */
    public double getFRAngleToTarget() {
        double angle = getRoboPose2d().getRotation().getDegrees() - LimelightHelpers.getTX("");
        if(hasTarget()) {
        
        return angle;
        }
        else return 0;
    }

    public void toggleLockTargetInAuto() {
        lockTargetInAuto = !lockTargetInAuto;
    }

    public double getRotationSpeedForTarget() {
        if(targetID != gamePieceIDs.kSpeakerID)
            return -aimFF.calculate(Math.toRadians(getAngleToTarget())) + aimPID.calculate(Math.toRadians(getAngleToTarget()));
        else
            return aimFF.calculate(getSpeakerAimTargetAngle() - getRoboPose2d().getRotation().getRadians());// + aimPID.calculate(getRoboPose2d().getRotation().getRadians(), getSpeakerAimTargetAngle());
    }

    /**
     * Resets Orientation of the robot
     */
    public void resetGyro() {
        m_gyro.setYaw(0);
    }

    /**
     * Resets robot position on the field
     */
    public void resetOdo() {
        m_odometry.resetPosition(getGyroYawRotation2d(), getModulePositions(), new Pose2d(new Translation2d(3, 7), new Rotation2d()));
    }

    /**
     * Resets Odometry using a specific Pose2d
     * @param pose
     */
    public void resetOdo(Pose2d pose) {
        //System.out.println(getGyroYawRotation2d().getDegrees());
        //System.out.println(pose.getX() + " " + pose.getY() + " " + pose.getRotation().getDegrees());
        m_odometry.resetPosition(getGyroYawRotation2d(), getModulePositions(), pose);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_chassisSpeeds;
    }

    public DirectionOption getDirectionOption() {
        return this.m_forwardDirection;
    }

    public Command setDirectionOptionCommand(DirectionOption option) {
        return runOnce(() -> {
            this.m_forwardDirection = option;
        });
    }
    
    public boolean getFieldRelative() {
        return fieldRelative;
    }

    public Command toggleFieldRelativeCommand() {
        return runOnce(() -> {
            fieldRelative = !fieldRelative;
            m_fieldRelativeWidget.getEntry().setBoolean(fieldRelative);
        });
    }

    /**
     * Module positions in the form of SwerveModulePositions (Module orientation and the distance the wheel has travelled across the ground)
     * @return SwerveModulePosition[]
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
        };
    }

    /**
     * Get the yaw of gyro in Rotation2d form
     * 
     * @return chasis angle in Rotation2d
     */
    public Rotation2d getGyroYawRotation2d() {
        // return Rotation2d.fromDegrees(MathUtil.inputModulus(m_gyro.getYaw(), -180, 180));
        return Rotation2d.fromDegrees(m_gyro.getYaw());
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rotSpeed      Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rotSpeed) {
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, getGyroYawRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxPossibleSpeed);

        // passing back the math from kinematics to the swerves themselves.
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);

        updateOdometry();
    }

    boolean speakerMode = true;

    public void toggleMode() {
        speakerMode = !speakerMode;
    }
    
    /**
     * Pathplanner uses this method in order to interface with our Drivetrain.
     * @param chassisSpeeds Robot relative ChassisSpeeds of the robot containing X, Y, and Rotational Velocities.
     */
    public void driveInAuto(ChassisSpeeds chassisSpeeds) {
        //SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        //comment: if speakermode, use speakeraim, if not speakermode, use piece aim. if using piece aim and dont see a piece, follow normal path.
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
                lockTargetInAuto
                        ? ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds.vxMetersPerSecond, 
                            chassisSpeeds.vyMetersPerSecond, 
                            hasTarget() ? getRotationSpeedForTarget() : chassisSpeeds.omegaRadiansPerSecond, //comment
                            getGyroYawRotation2d())
                        : chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxPossibleSpeed);
        //
        SmartDashboard.putNumber("desired X Speed", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("desired Y Speed", chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("desired Rot Speed", Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond));

        // passing back the math from kinematics to the swerves themselves.
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
        updateOdometry();
    }

    /**
     * What this code essentially does is when you're holding down the left button (in robot), just drive like normal until you see a game piece. 
     * once you see it, its locked. from this point, you may drive around as normal, but the aimlock will handle the robot's orientation. 
     * If you then press the left trigger (in robot), the robot will "hard lock" onto the piece, and you will only be able
     *  to go forwards and backwards (robot oriented), in order to perfectly pick up the piece.
     *  With proper tuning it will be able to lock, and stay locked onto the tag/piece no matter how fast we are driving. 
     */
    public void lockPiece(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative, boolean hardLocked) {
        SwerveModuleState[] swerveModuleStates; //MAKE SURE swervestates can be init like this with this kinda array
        if(hasTarget() || getTarget() == gamePieceIDs.kSpeakerID) {
                rotSpeed = getRotationSpeedForTarget();
                SmartDashboard.putNumber("rotSpeed deg", Math.toDegrees(rotSpeed));
                if(hardLocked) {
                        swerveModuleStates = m_kinematics.toSwerveModuleStates(
                                new ChassisSpeeds(xSpeed, 0, rotSpeed)
                        );
                }
                else {
                        swerveModuleStates = m_kinematics.toSwerveModuleStates(
                                ChassisSpeeds.fromFieldRelativeSpeeds(
                                        xSpeed, ySpeed, rotSpeed, getGyroYawRotation2d()
                                )
                        );
                }
        }
        else {
                swerveModuleStates = m_kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed,
                                getGyroYawRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxPossibleSpeed);

        // passing back the math from kinematics to the swerves themselves.
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);

        SmartDashboard.putNumber("desired X Speed", xSpeed);
        SmartDashboard.putNumber("desired Y Speed", ySpeed);
        SmartDashboard.putNumber("desired Rot Speed", Math.toDegrees(rotSpeed));
        updateOdometry();
    }


    public Pose2d getRoboPose2d() {
        return m_odometry.getPoseMeters();
    }

    /**
     * THIS IS IN DEGREES
     * Triangulates position of robot knowing the distance between two april tags seen by the camera.
     * @param length
     * @param angle1
     * @param angle2
     * @return
     */
    public Pose2d calcRoboPose2dWithVision(double length, double angle1, double angle2) {
        double L = length; //dist between the two april tags
        double a1 = angle1; //angle (from the camera) of the close april tag (a1) and the far april tag (a2)
        double a2 = angle2;
        double gyroOffset = 0;
        double roboAngle = -m_gyro.getYaw() + gyroOffset; //angle of the robot (0 degrees = facing the drivers)

        double X = (L * Math.sin(Math.toRadians(90 + roboAngle + a2)) * Math.sin(Math.toRadians(90 - roboAngle - a1)))
                        /Math.sin(Math.toRadians(Math.abs(a2 - a1)));

        double Y = (L * Math.sin(Math.toRadians(90 + roboAngle + a2)) * Math.cos(Math.toRadians(90 - roboAngle - a1)))
                        /Math.sin(Math.toRadians(Math.abs(a2 - a1)));

        return new Pose2d(X, Y, getGyroYawRotation2d());
    }

    public void stopDriving() {
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(
                0,
                0,
                0,
                getGyroYawRotation2d()));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxPossibleSpeed);

        // passing back the math from kinematics to the swerves themselves.
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        SmartDashboard.putNumber("limelight FR target angle", getRoboPose2d().getRotation().getDegrees() - LimelightHelpers.getTX(""));
        SmartDashboard.putNumber("angle to speaker", getAngleToSpeaker());
        SmartDashboard.putNumber("ID", LimelightHelpers.getFiducialID(""));
        m_odometry.update(
                getGyroYawRotation2d(),
                getModulePositions());
        
        // getting velocity vectors from each module
        SwerveModuleState frontLeftState = m_frontLeft.getState();
        SwerveModuleState frontRightState = m_frontRight.getState();
        SwerveModuleState backLeftState = m_backLeft.getState();
        SwerveModuleState backRightState = m_backRight.getState();

        // Converting module speeds to chassis speeds
        m_chassisSpeeds = m_kinematics.toChassisSpeeds(
                frontLeftState, frontRightState, backLeftState, backRightState);
        robotFieldPosition = getRoboPose2d();
    }
}
