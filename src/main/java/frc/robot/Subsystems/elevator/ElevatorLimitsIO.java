package frc.robot.Subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorLimitsIO {
    @AutoLog
    public static class ElevatorLimitsIOInputs {
        public boolean touchingBottom = false;
        public boolean touchingTop = false;
    }

    public default void updateInputs(ElevatorLimitsIOInputs inputs) {
    }
}
