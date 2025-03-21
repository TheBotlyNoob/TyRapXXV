package frc.robot.Subsystems.coral;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants;

public class CoralWristIOSpark implements CoralWristIO {
    private final SparkMax motor;

    public CoralWristIOSpark() {
        motor = new SparkMax(Constants.MechID.kCoralWristCanId, MotorType.kBrushed);
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
