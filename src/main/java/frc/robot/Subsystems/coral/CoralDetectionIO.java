package frc.robot.Subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

public interface CoralDetectionIO {
    @AutoLog
    public static class CoralDetectionIOInputs {
        public boolean hasCoral = false;
    }

    public default void updateInputs(CoralDetectionIOInputs inputs) {
    }
}
