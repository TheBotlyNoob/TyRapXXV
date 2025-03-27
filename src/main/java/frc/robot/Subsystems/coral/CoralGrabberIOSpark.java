package frc.robot.Subsystems.coral;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class CoralGrabberIOSpark implements CoralGrabberIO {
    private final SparkMax motor = new SparkMax(Constants.MechID.kCoralWheelCanId, MotorType.kBrushless);

    public CoralGrabberIOSpark() {
        motor.configure(new SparkMaxConfig().apply(Constants.SparkConstants.defaultSignalConf),
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(CoralGrabberIOInputs inputs) {
        inputs.relativeEncoderPosition = new Rotation2d(Units.Rotations.of(motor.getEncoder().getPosition()));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }
}
