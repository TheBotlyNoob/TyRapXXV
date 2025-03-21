package frc.robot.Subsystems.coral;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class CoralGrabberIOSpark implements CoralGrabberIO {
    private final SparkMax motor = new SparkMax(Constants.MechID.kCoralWheelCanId, MotorType.kBrushless);

    public CoralGrabberIOSpark() {
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
