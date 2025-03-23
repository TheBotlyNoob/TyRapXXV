package frc.robot.Subsystems.drive;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.Modules;

public class SwerveModuleIOSpark implements SwerveModuleIO {
    /**
     * The drive motor is responsible for the actual power across the ground e.g. to
     * make it move forward/backward
     */
    SparkBase m_driveMotor;
    /**
     * The turning/steering motor is responsible for the orientation of the wheel
     * e.g. to make it turn
     */
    SparkBase m_turningMotor;

    /**
     * This does not control the motor. This just records the orientation of the
     * module.
     * IMPORTANT NOTE: Ensure that the encoder and motor both agree which
     * direction is positive.
     * We faced an issue where the motor oscillated inexplicably and we traced it
     * back to the motor needing to be inverted to match with the encoder.
     */
    CANcoder m_turningEncoder;

    /**
     * Each turning motor has an encoder. Each motor was mounted by people at
     * seemingly random offsets. We have to accommodate for these
     * offsets so that calls to getAbsolutePosition all return the same value
     * regardless of motor i.e. make them consider the same reference point as "0"
     */
    double m_encoderOffset;

    public SwerveModuleIOSpark(int driveChannel, int turnChannel, int turningEncoderChannel,
            double encoderOffset,
            boolean sparkFlex,
            boolean invertDrive) {
        SparkBaseConfig driveConfig;
        /*
         * Set up the drive motor
         */
        if (sparkFlex) {
            m_driveMotor = new SparkFlex(driveChannel, MotorType.kBrushless);
            driveConfig = new SparkFlexConfig();
        } else {
            m_driveMotor = new SparkMax(driveChannel, MotorType.kBrushless);
            driveConfig = new SparkMaxConfig();
        }

        // TODO: moved this code outside of the if. Did that change behavior? (it
        // shouldn't)
        m_driveMotor.getEncoder().setPosition(0);
        driveConfig.smartCurrentLimit(60); // m_driveMotor.setSmartCurrentLimit(40);
        driveConfig.encoder.positionConversionFactor(Modules.kDriveEncoderRot2Meter); // m_driveMotor.getEncoder().setPositionConversionFactor(Modules.kDriveEncoderRot2Meter);
        driveConfig.encoder.velocityConversionFactor(Modules.kDriveEncoderRPM2MeterPerSec); // m_driveMotor.getEncoder().setVelocityConversionFactor(Modules.kDriveEncoderRPM2MeterPerSec);
        driveConfig.idleMode(IdleMode.kBrake); // m_driveMotor.setIdleMode(IdleMode.kBrake);
        driveConfig.inverted(invertDrive); // m_driveMotor.setInverted(false);
        m_driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        /*
         * Set up the turning motor. We had to invert the turning motor so it agreed
         * with the turning encoder which direction was positive
         */
        m_turningMotor = new SparkMax(turnChannel, MotorType.kBrushless);

        SparkMaxConfig turningConfig = new SparkMaxConfig();
        turningConfig.inverted(DrivetrainConstants.kInvertTurn); // m_turningMotor.setInverted(true);
        turningConfig.smartCurrentLimit(60); // m_turningMotor.setSmartCurrentLimit(40);
        turningConfig.idleMode(IdleMode.kBrake); // m_turningMotor.setIdleMode(IdleMode.kBrake);
        LimitSwitchConfig limitConfig = new LimitSwitchConfig();
        limitConfig.forwardLimitSwitchEnabled(false);
        limitConfig.reverseLimitSwitchEnabled(false);
        turningConfig.apply(limitConfig);
        m_turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_encoderOffset = encoderOffset;

        /*
         * Set up and configure the turning encoder. Additional config is required to
         * ensure that units are correct
         */
        m_turningEncoder = new CANcoder(turningEncoderChannel);

        // Want absolute readings [0.5, 0.5)
        MagnetSensorConfigs turningEncoderMagnetSensorConfigs = new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(0.5) // .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
        m_turningEncoder.getConfigurator().apply(turningEncoderMagnetSensorConfigs);
        m_turningEncoder.clearStickyFaults();

    }

    @Override
    public void updateInputs(SwerveModuleIO.SwerveModuleIOInputs inputs) {
        inputs.driveMotorVelocity = Units.MetersPerSecond.of(m_driveMotor.getEncoder().getVelocity());
        inputs.driveMotorAngularVelocity = Units.RPM.of(inputs.driveMotorVelocity
                .in(Units.MetersPerSecond) / Constants.Modules.kDriveEncoderRPM2MeterPerSec);
        inputs.drivingMotorDistance = Units.Meters
                .of(m_driveMotor.getEncoder().getPosition());
        inputs.drivingMotorPosition = Units.Rotations
                .of(/* Meters */m_driveMotor.getEncoder().getPosition() / Constants.Modules.kDriveEncoderRot2Meter);

        inputs.turningMotorPosition = m_turningEncoder.getAbsolutePosition().getValue(); // angleModulus
                                                                                         // ensures
        // that the
        // angle within -PI to PI
        inputs.turningMotorAngularVelocity = m_turningEncoder.getVelocity().getValue();
    }

    @Override
    public void setDriveVoltage(Voltage voltage) {
        m_driveMotor.setVoltage(voltage);
    }

    @Override
    public void setTurnVoltage(Voltage voltage) {
        m_turningMotor.setVoltage(voltage);
    }
}
