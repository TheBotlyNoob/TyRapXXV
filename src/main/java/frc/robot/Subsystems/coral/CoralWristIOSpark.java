package frc.robot.Subsystems.coral;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants;

public class CoralWristIOSpark implements CoralWristIO {
    private final SparkMax motor = new SparkMax(Constants.MechID.kCoralWristCanId, MotorType.kBrushed);

    public CoralWristIOSpark() {
        motor.configure(new SparkMaxConfig().apply(Constants.SparkConstants.defaultSignalConf),
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(CoralWristIOInputs inputs) {
        inputs.absoluteEncoderPosition = new Rotation2d(Units.Rotations.of(motor.getAbsoluteEncoder().getPosition()));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }
}
