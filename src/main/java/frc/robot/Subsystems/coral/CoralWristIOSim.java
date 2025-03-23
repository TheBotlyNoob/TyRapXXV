package frc.robot.Subsystems.coral;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class CoralWristIOSim implements CoralWristIO {
    private DCMotor gearbox = DCMotor.getNeo550(1);
    private DCMotorSim motor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, 4.0, 1 / Constants.Modules.kDriveMotorGearRatio), gearbox);

    public CoralWristIOSim() {
    }

    @Override
    public void updateInputs(CoralWristIOInputs inputs) {
        inputs.absoluteEncoderPosition = new Rotation2d(motor.getAngularPosition());
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setInputVoltage(voltage.in(Units.Volts));
    }
}
