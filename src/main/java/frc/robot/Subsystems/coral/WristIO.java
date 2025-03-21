package frc.robot.Subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;

public interface WristIO {
    @AutoLog
    public class WristIOInputs {
        public Rotation2d absoluteEncoderPosition = new Rotation2d(Units.Radians.of(0.0));
    }

    public default void updateInputs(WristIOInputs inputs) {
    }

    public default void setVoltage(Voltage voltage) {
    }
}
