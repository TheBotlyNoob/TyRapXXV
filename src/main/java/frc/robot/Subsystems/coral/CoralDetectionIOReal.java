package frc.robot.Subsystems.coral;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class CoralDetectionIOReal implements CoralDetectionIO {
    private final DigitalInput m_sensor = new DigitalInput(Constants.SensorID.kIRSensorPort);

    public CoralDetectionIOReal() {
    }

    @Override
    public void updateInputs(CoralDetectionIOInputs inputs) {
        // our IR sensor is TRUE when unblocked, and FALSE when blocked
        inputs.hasCoral = !m_sensor.get();
    }
}
