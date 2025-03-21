package frc.robot.Subsystems.coral;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class WristIOSim implements WristIO {
    private DCMotor gearbox = DCMotor.getNeo550(1);
    private DCMotorSim motor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, 4.0, 1 / Constants.Modules.kDriveMotorGearRatio), gearbox);

    public WristIOSim() {
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.absoluteEncoderPosition = new Rotation2d(motor.getAngularPosition());
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setInputVoltage(voltage.in(Units.Volts));
    }
}
