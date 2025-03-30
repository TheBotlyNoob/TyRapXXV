package frc.robot.Subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberPneumaticsIO {
    @AutoLog
    public static class ClimberPneumaticsIOInputs {
        public ArmState armState = ArmState.OFF;
        public GrabberState grabberState = GrabberState.OFF;
        public RampState rampState = RampState.OFF;
    }

    public static enum ArmState {
        EXTENDED,
        RETRACTED,
        OFF
    }

    public static enum GrabberState {
        OPEN,
        CLOSED,
        OFF
    }

    public static enum RampState {
        UP,
        DOWN,
        OFF
    }

    public default void updateInputs(ClimberPneumaticsIOInputs inputs) {
    }

    public default void setArmState(ArmState state) {
    }

    public default void setGrabberState(GrabberState state) {
    }

    public default void setRampState(RampState state) {
    }
}
