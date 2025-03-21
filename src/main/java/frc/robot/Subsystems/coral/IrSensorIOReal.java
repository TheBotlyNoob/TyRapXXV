package frc.robot.Subsystems.coral;

import edu.wpi.first.wpilibj.DigitalInput;

public class IrSensorIOReal implements IrSensorIO {
    private final DigitalInput m_sensor;

    public IrSensorIOReal(DigitalInput sensor) {
        m_sensor = sensor;
    }

    @Override
    public void updateInputs(IrSensorIOInputs inputs) {
        // our IR sensor is TRUE when unblocked, and FALSE when blocked
        inputs.isBlocked = !m_sensor.get();
    }
}
