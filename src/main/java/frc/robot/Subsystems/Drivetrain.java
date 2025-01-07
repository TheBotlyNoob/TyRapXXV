// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ID;
import frc.robot.Constants.Offsets;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveTrain;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
    public static final double kMaxPossibleSpeed = 1.5; // meters per second
    public static final double kMaxAngularSpeed = 1.5 * Math.PI; // per second

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

    private ExecutorService executorService = Executors.newFixedThreadPool(4);

    public Drivetrain(Pigeon2 gyro) {
        // Zero at beginning of match. Zero = whatever direction the robot (more
        // specifically the gyro) is facing
        this.m_gyro = gyro;
        this.resetGyro();

        m_odometry = new SwerveDriveOdometry(
                m_kinematics,
                getGyroYawRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_backLeft.getPosition(),
                        m_backRight.getPosition()
                });
    }

    public Pigeon2 getGyro() {
        return m_gyro;
    }

    /**
     * Resets Orientation of the robot
     */
    public void resetGyro() {
        m_gyro.setYaw(0);
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

    /**
     * Resets robot position on the field
     */
    public void resetOdo() {
        if (DriverStation.getRawAllianceStation().equals(AllianceStationID.Blue1)
                || DriverStation.getRawAllianceStation().equals(AllianceStationID.Blue2)
                || DriverStation.getRawAllianceStation().equals(AllianceStationID.Blue3)) {
            m_odometry.resetPosition(getGyroYawRotation2d(), getModulePositions(),
                    new Pose2d(new Translation2d(getRoboPose2d().getX(), getRoboPose2d().getY()),
                            new Rotation2d()));
        } else {
            m_odometry.resetPosition(getGyroYawRotation2d(), getModulePositions(),
                    new Pose2d(new Translation2d(getRoboPose2d().getX(), getRoboPose2d().getY()),
                            new Rotation2d(Math.PI)));
        }
    }

    /**
     * Resets Odometry using a specific Pose2d
     * 
     * @param pose
     */
    public void resetOdo(Pose2d pose) {
        if (pose != null) {
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
        m_fieldRelativeWidget.getEntry().setBoolean(fieldRelative);
    }

    public Command setFieldRelativeCommand(boolean isFieldRelative) {
        return runOnce(() -> {
            this.setFieldRelative(isFieldRelative);
        });
    }

    /**
     * Module positions in the form of SwerveModulePositions (Module orientation and
     * the distance the wheel has travelled across the ground)
     * 
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
        return Rotation2d.fromDegrees(m_gyro.getYaw().getValue());
    }

    private double driveMultiplier = 1;

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
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed,
                                getGyroYawRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxPossibleSpeed);

        // passing back the math from kinematics to the swerves themselves.
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

        // m_frontLeft.setDesiredState(swerveModuleStates[0]);
        // m_frontRight.setDesiredState(swerveModuleStates[1]);
        // m_backLeft.setDesiredState(swerveModuleStates[2]);
        // m_backRight.setDesiredState(swerveModuleStates[3]);

        SmartDashboard.putNumber("desired X speed", xSpeed);
        SmartDashboard.putNumber("desired Y speed", ySpeed);
        // this.layout.setDesiredRotSpeed(Math.toDegrees(rotSpeed));
    }

    public Pose2d getRoboPose2d() {
        return m_odometry.getPoseMeters();
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
    }

    @Override
    public void periodic() {
        updateOdometry();
    }
}
