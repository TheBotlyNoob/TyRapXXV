package frc.robot.Subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;

public interface CoralGrabberIO {
    @AutoLog
    public static class CoralGrabberIOInputs {
        public Rotation2d relativeEncoderPosition = new Rotation2d(Units.Radians.of(0.0));
    }

    public default void updateInputs(CoralGrabberIOInputs inputs) {
    }

    public default void setVoltage(Voltage voltage) {
    }

    public default void setSpeed(double speed) {
    }
}
