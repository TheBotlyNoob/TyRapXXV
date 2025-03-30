package frc.robot.Subsystems.climber;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberStingerIO {
    @AutoLog
    public static class ClimberStingerIOInputs {
        public Angle dutyCycleEncoderPosition = Units.Rotations.of(0.0);
    }

    public default void updateInputs(ClimberStingerIOInputs inputs) {}

    public default void setVoltage(double voltage) {}
}
