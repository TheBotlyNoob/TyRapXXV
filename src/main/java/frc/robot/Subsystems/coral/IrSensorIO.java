package frc.robot.Subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

public interface IrSensorIO {
    @AutoLog
    public class IrSensorIOInputs {
        public boolean isBlocked = false;
    }

    public default void updateInputs(IrSensorIOInputs inputs) {
    }
}
