// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.stream.Stream;

public class Drivetrain extends SubsystemBase {

    public static final double kMaxAngularSpeed = 1.5 * Math.PI; // per second

    protected final Translation2d m_frontLeftLocation = new Translation2d(
            DrivetrainConstants.kDistanceMiddleToFrontMotor * DrivetrainConstants.kXForward,
            DrivetrainConstants.kDistanceMiddleToSideMotor * DrivetrainConstants.kYLeft);
    protected final Translation2d m_frontRightLocation = new Translation2d(
            DrivetrainConstants.kDistanceMiddleToFrontMotor * DrivetrainConstants.kXForward,
            DrivetrainConstants.kDistanceMiddleToSideMotor * DrivetrainConstants.kYRight);
    protected final Translation2d m_backLeftLocation = new Translation2d(
            DrivetrainConstants.kDistanceMiddleToFrontMotor * DrivetrainConstants.kXBackward,
            DrivetrainConstants.kDistanceMiddleToSideMotor * DrivetrainConstants.kYLeft);
    protected final Translation2d m_backRightLocation = new Translation2d(
            DrivetrainConstants.kDistanceMiddleToFrontMotor * DrivetrainConstants.kXBackward,
            DrivetrainConstants.kDistanceMiddleToSideMotor * DrivetrainConstants.kYRight);

    protected final SwerveModule m_frontLeft;
    protected final SwerveModule m_frontRight;
    protected final SwerveModule m_backLeft;
    protected final SwerveModule m_backRight;

    protected final SwerveModule[] m_swerveModules = new SwerveModule[4];

    protected final GyroIO m_gyroIo;
    protected final GyroIOInputsAutoLogged m_gyroInputs = new GyroIOInputsAutoLogged();

    @AutoLogOutput
    protected boolean enableVisionPoseInputs = true;

    @AutoLogOutput
    protected boolean fieldRelative = true;

    /**
     * The order that you initialize these is important! Later uses of functions
     * like toSwerveModuleStates will return the same order that these are provided.
     * See
     * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
     */
    protected final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    protected final SwerveDrivePoseEstimator m_odometry;
    protected int counter = 0;

    @AutoLogOutput(key = "Drivetrain/ChassisSpeeds")
    protected ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
    @AutoLogOutput
    protected ChassisSpeeds commandedChassisSpeeds = new ChassisSpeeds();

    protected ExecutorService executorService = Executors.newFixedThreadPool(4);

    private SwerveModule makeSwerveMod(String name, SwerveModuleIO io) {
        return new SwerveModule(io,
                name,
                DrivetrainConstants.turnPID,
                DrivetrainConstants.drivePID,
                DrivetrainConstants.turnFeedForward,
                DrivetrainConstants.driveFeedForward);
    }

    public Drivetrain(GyroIO gyro, SwerveModuleIO frontLeft, SwerveModuleIO frontRight, SwerveModuleIO backLeft,
            SwerveModuleIO backRight) {
        // Zero at beginning of match. Zero = whatever direction the robot (more
        // specifically the gyro) is facing
        this.m_gyroIo = gyro;

        m_frontLeft = makeSwerveMod("FrontLeft", frontLeft);
        m_frontRight = makeSwerveMod("FrontRight", frontRight);
        m_backLeft = makeSwerveMod("BackLeft", backLeft);
        m_backRight = makeSwerveMod("BackRight", backRight);

        m_swerveModules[0] = m_frontLeft;
        m_swerveModules[1] = m_frontRight;
        m_swerveModules[2] = m_backLeft;
        m_swerveModules[3] = m_backRight;

        // TODO: allow PID to be tuned--make a new IO for that.
        m_odometry = new SwerveDrivePoseEstimator(
                m_kinematics,
                getGyroYawRotation2d(),
                getModulePositions(),
                new Pose2d(0.0, 0.0, new Rotation2d(0.0)));

        // Load the RobotConfig from the PathPlanner GUI settings
        RobotConfig ppConfig;
        try {
            ppConfig = RobotConfig.fromGUISettings();

            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getRoboPose2d, // Robot pose supplier
                    this::resetOdo, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    this::driveChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                    new PPHolonomicDriveController( // HolonomicPathFollowerConfig
                            new PIDConstants(DrivetrainConstants.drivePID[0], // Translation PID constants
                                    DrivetrainConstants.drivePID[1],
                                    DrivetrainConstants.drivePID[2]),
                            new PIDConstants(DrivetrainConstants.turnPID[0], // Rotation PID constants
                                    DrivetrainConstants.turnPID[1],
                                    DrivetrainConstants.turnPID[2])),
                    ppConfig,
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
                    this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
    }

    public void resetStartingPose(Pose2d newPose) {
        m_odometry.resetPosition(getGyroYawRotation2d(), getModulePositions(), newPose);
    }

    public void resetStartingTranslation(Translation2d newTranslation) {
        m_odometry.resetTranslation(newTranslation);
    }

    /**
     * Resets Orientation of the robot
     */
    public void resetGyro() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Blue) {
                System.out.println("Initializing gyro to 180 for BLUE");
                m_gyroIo.setYaw(Rotation2d.fromDegrees(180.0));
            } else {
                System.out.println("Initializing gyro to 0 for RED");
                m_gyroIo.setYaw(Rotation2d.fromDegrees(0.0));
            }
        } else {
            System.out.println("Initializing gyro to 0 for default");
            m_gyroIo.setYaw(Rotation2d.fromDegrees(0.0));
        }
    }

    public SwerveModule getBackLeftSwerveModule() {
        return m_backLeft;
    }

    public SwerveModule getBackRightSwerveModule() {
        return m_backRight;
    }

    public SwerveModule getFrontLeftSwerveModule() {
        return m_frontLeft;
    }

    public SwerveModule getFrontRightSwerveModule() {
        return m_frontRight;
    }

    public SwerveModule[] getSwerveModules() {
        return m_swerveModules;
    }

    /**
     * Module positions in the form of SwerveModulePositions (Module orientation and
     * the distance the wheel has travelled across the ground)
     * 
     * @return SwerveModulePosition[]
     */
    public SwerveModulePosition[] getModulePositions() {
        return Stream.of(getSwerveModules()).map((mod) -> mod.getPosition())
                .toArray(SwerveModulePosition[]::new);
    }

    /**
     * Resets robot position on the field
     */
    public void resetOdo() {
        System.out.println("Calling resetOdo with no arguments");
        if (DriverStation.getAlliance().map((al) -> al == Alliance.Blue).orElse(false)) {
            m_odometry.resetPosition(getGyroYawRotation2d(), getModulePositions(),
                    new Pose2d(new Translation2d(getRoboPose2d().getX(), getRoboPose2d().getY()),
                            new Rotation2d()));
        } else {
            m_odometry.resetPosition(getGyroYawRotation2d(), getModulePositions(),
                    new Pose2d(new Translation2d(getRoboPose2d().getX(), getRoboPose2d().getY()),
                            new Rotation2d()));
        }
    }

    /**
     * Resets Odometry using a specific Pose2d
     * 
     * @param pose
     */
    public void resetOdo(Pose2d pose) {
        if (pose != null) {
            System.out.println("Calling resetOdo with pose " + pose);
            m_odometry.resetPosition(getGyroYawRotation2d(), getModulePositions(), pose);
        }
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_chassisSpeeds;
    }

    public boolean getFieldRelative() {
        return fieldRelative;
    }

    public void setFieldRelative(boolean isFieldRelative) {
        fieldRelative = isFieldRelative;
    }

    public Command toggleFieldRelativeCommand() {
        return runOnce(() -> {
            this.setFieldRelative(!this.fieldRelative);
        });
    }

    /**
     * Get the yaw of gyro in Rotation2d form
     * 
     * @return chasis angle in Rotation2d
     */
    public Rotation2d getGyroYawRotation2d() {
        return m_gyroInputs.yawPosition;
    }

    public Rotation2d getPlayerStationRelativeYaw2d() {
        Rotation2d rot = m_gyroInputs.yawPosition;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                rot = rot.rotateBy(Rotation2d.k180deg);
            } else {
            }
        } else {
            System.err.println("Alliance not set");
        }
        return rot;
    }

    public ChassisSpeeds getCommandeChassisSpeeds() {
        return commandedChassisSpeeds;
    }

    protected double driveMultiplier = 1;

    public void setDriveMult(double mult) {
        driveMultiplier = mult;
    }

    public Command setDriveMultCommand(double mult) {
        return Commands.runOnce(() -> setDriveMult(mult));
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
        xSpeed = xSpeed * driveMultiplier;
        ySpeed = ySpeed * driveMultiplier;
        rotSpeed = rotSpeed * driveMultiplier;

        ChassisSpeeds chassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed,
                        getPlayerStationRelativeYaw2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        driveChassisSpeeds(chassisSpeeds);
    }

    public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        commandedChassisSpeeds = chassisSpeeds;
    }

    @AutoLogOutput
    public Pose2d getRoboPose2d() {
        return m_odometry.getEstimatedPosition();
    }

    public void setEnableVisionPoseInputs(boolean enableVisionPoseInputs) {
        this.enableVisionPoseInputs = enableVisionPoseInputs;
    }

    public void stopDriving() {
        commandedChassisSpeeds = new ChassisSpeeds();
        // Get velocity and position from each module
        SwerveModuleState frontLeftState = m_frontLeft.getState();
        SwerveModuleState frontRightState = m_frontRight.getState();
        SwerveModuleState backLeftState = m_backLeft.getState();
        SwerveModuleState backRightState = m_backRight.getState();
        // Command each module to zero speed while maintaining its current
        // angle
        frontLeftState.speedMetersPerSecond = 0.0;
        m_frontLeft.setDesiredState(frontLeftState);
        frontRightState.speedMetersPerSecond = 0.0;
        m_frontRight.setDesiredState(frontRightState);
        backLeftState.speedMetersPerSecond = 0.0;
        m_backLeft.setDesiredState(backLeftState);
        backRightState.speedMetersPerSecond = 0.0;
        m_backRight.setDesiredState(backRightState);
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        Rotation2d rotationYaw = getGyroYawRotation2d();
        m_odometry.update(
                rotationYaw,
                getModulePositions());

        // getting velocity vectors from each module
        SwerveModuleState frontLeftState = m_frontLeft.getState();
        SwerveModuleState frontRightState = m_frontRight.getState();
        SwerveModuleState backLeftState = m_backLeft.getState();
        SwerveModuleState backRightState = m_backRight.getState();

        // Converting module speeds to chassis speeds
        m_chassisSpeeds = m_kinematics.toChassisSpeeds(
                frontLeftState, frontRightState, backLeftState, backRightState);
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        if (!enableVisionPoseInputs) {
            return;
        }

        m_odometry.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    @Override
    public void periodic() {
        m_gyroIo.updateInputs(m_gyroInputs);
        Logger.processInputs("Drivetrain/Gyro", m_gyroInputs);

        for (SwerveModule module : getSwerveModules()) {
            module.periodic();
        }

        updateOdometry();

        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(commandedChassisSpeeds);

        Logger.recordOutput("Drivetrain/SwerveModuleStates/Desired", swerveModuleStates);
        Logger.recordOutput("Drivetrain/SwerveModuleStates/Real",
                Stream.of(getSwerveModules()).map(SwerveModule::getState).toArray(SwerveModuleState[]::new));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivetrainConstants.kMaxPossibleSpeed);

        // passing back the math from kinematics to the swerves themselves.
        // using the executor service to run these in parallel
        CountDownLatch latch = new CountDownLatch(4);
        executorService.execute(() -> {
            m_frontLeft.setDesiredState(swerveModuleStates[0]);
            latch.countDown();
        });
        executorService.execute(() -> {
            m_frontRight.setDesiredState(swerveModuleStates[1]);
            latch.countDown();
        });
        executorService.execute(() -> {
            m_backLeft.setDesiredState(swerveModuleStates[2]);
            latch.countDown();
        });
        executorService.execute(() -> {
            m_backRight.setDesiredState(swerveModuleStates[3]);
            latch.countDown();
        });
        try {
            latch.await();
        } catch (InterruptedException e) {
            // Pass
        }
    }
}
