// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.*;

public class SwerveModule {

    private static final double kModuleMaxAngularVelocity = 32 * Math.PI;
    private static final double kModuleMaxAngularAcceleration = 40 * Math.PI; // radians per second squared this was
                                                                              // 2PI

    /**
     * Identification for what motor this is
     */
    private final String m_swerveModuleName;

    private final PIDController m_drivePIDController;

    private final ProfiledPIDController m_turningPIDController;

    private double m_turningLastSpeed = 0;
    private double m_turningLastTime = Timer.getTimestamp();
    private double actualTurnVelocityRps = 0.0;

    private final SimpleMotorFeedforward m_driveFeedforward;
    private final SimpleMotorFeedforward m_turnFeedforward;

    protected final SwerveModuleIO m_io;
    protected final SwerveModuleIOInputsAutoLogged m_inputs = new SwerveModuleIOInputsAutoLogged();

    private SwerveModuleState m_desiredState = new SwerveModuleState();

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
     * and turning encoder.
     *
     * @param driveMotorChannel      PWM output for the drive motor.
     * @param turningMotorChannel    PWM output for the turning motor.
     * @param driveEncoderChannelA   DIO input for the drive encoder channel A
     * @param driveEncoderChannelB   DIO input for the drive encoder channel B
     * @param turningEncoderChannelA DIO input for the turning encoder channel A
     * @param turningEncoderChannelB DIO input for the turning encoder channel B
     */
    public SwerveModule(
            SwerveModuleIO io,
            String swerveModuleID,
            double[] steerPID,
            double[] drivePID,
            double[] turnFeedForward,
            double[] driveFeedForward) {
        m_io = io;

        m_swerveModuleName = swerveModuleID;

        m_drivePIDController = new PIDController(
                drivePID[0],
                drivePID[1],
                drivePID[2]);

        m_turningPIDController = new ProfiledPIDController(
                steerPID[0],
                steerPID[1],
                steerPID[2],
                new TrapezoidProfile.Constraints(
                        kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        m_turnFeedforward = new SimpleMotorFeedforward(turnFeedForward[0], turnFeedForward[1]);
        m_driveFeedforward = new SimpleMotorFeedforward(driveFeedForward[0], driveFeedForward[1]);

    }

    /**
     * Returns the current Swerve Module state. This includes info about the module
     * wheel's speed per second and the angle of the module. Units are based on the
     * configuration of the motors and encoders
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_inputs.driveMotorVelocity,
                new Rotation2d(m_inputs.turningMotorPosition));
    }

    /**
     * Returns the current position of the module. This includes info about the
     * distance measured by the wheel and the angle of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                m_inputs.drivingMotorDistance,
                new Rotation2d(m_inputs.turningMotorPosition));
    }

    public SwerveModuleState getDesiredState() {
        return m_desiredState;
    }

    public void resetDriveError() {
        m_drivePIDController.reset();
    }

    // Controls a simple motor's position using a SimpleMotorFeedforward
    // and a ProfiledPIDController
    public void goToPosition(double goalPosition) {
        double targetVelocity = m_turningPIDController.getSetpoint().velocity;
        double time = Timer.getFPGATimestamp();
        double targetAcceleration = (targetVelocity - this.m_turningLastSpeed)
                / (time - this.m_turningLastTime);

        double pidVal = m_turningPIDController.calculate(m_inputs.turningMotorPosition.in(Units.Radians), goalPosition);
        double FFVal = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity,
                targetAcceleration);

        m_io.setTurnVoltage(Units.Volts.of(pidVal + FFVal));
        this.m_turningLastSpeed = actualTurnVelocityRps;
        this.m_turningLastTime = time;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees or
        // PI/2 radians
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(m_inputs.turningMotorPosition));
        // SwerveModuleState state = desiredState;
        // state.optimize(new Rotation2d(getActualTurningPosition()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = m_drivePIDController.calculate(m_inputs.driveMotorVelocity.in(Units.MetersPerSecond),
                state.speedMetersPerSecond);

        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        this.goToPosition(state.angle.getRadians());

        if (desiredState.speedMetersPerSecond < 0.01) {
            m_io.setDriveVoltage(Units.Volts.of(0.0));
        } else {
            m_io.setDriveVoltage(Units.Volts.of(driveOutput + driveFeedforward));
        }
        m_desiredState = state;
    }

    public PIDController getDrivePidController() {
        return m_drivePIDController;
    }

    public SimpleMotorFeedforward getDriveFeedForward() {
        return m_driveFeedforward;
    }

    public ProfiledPIDController getTurnPidController() {
        return m_turningPIDController;
    }

    public SimpleMotorFeedforward getTurnFeedForward() {
        return m_turnFeedforward;
    }

    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs("Drivetrain/SwerveModule" + m_swerveModuleName, m_inputs);
    }
}
