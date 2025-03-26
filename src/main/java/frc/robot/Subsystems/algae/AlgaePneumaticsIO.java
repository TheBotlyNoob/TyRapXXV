package frc.robot.Subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaePneumaticsIO {
    public static enum AlgaeGrabberState {
        EXTENDED,
        RETRACTED,
        OFF
    }
    @AutoLog
    public static class AlgaePneumaticsIOInputs {
        public AlgaeGrabberState state = AlgaeGrabberState.OFF;
    }

    public default void updateInputs(AlgaePneumaticsIOInputs inputs) {
    }

    public default void setState(AlgaeGrabberState state) {
    }
}
