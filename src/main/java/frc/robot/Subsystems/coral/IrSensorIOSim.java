package frc.robot.Subsystems.coral;

import edu.wpi.first.wpilibj.DigitalInput;

public class IrSensorIOSim implements IrSensorIO {
    public final boolean isBlocked = false;

    public IrSensorIOSim() {
    }

    @Override
    public void updateInputs(IrSensorIOInputs inputs) {
        // our IR sensor is TRUE when unblocked, and FALSE when blocked
        inputs.isBlocked = isBlocked;
    }
}
