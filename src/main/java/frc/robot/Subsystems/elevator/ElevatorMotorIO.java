package frc.robot.Subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface ElevatorMotorIO {
    @AutoLog
    public static class ElevatorMotorIOInputs {
        public Angle leaderRelativeEncoderPosition = Units.Radians.of(0.0);
        public AngularVelocity leaderRelativeEncoderVelocity = Units.RadiansPerSecond.of(0.0);
    }

    public default void updateInputs(ElevatorMotorIOInputs inputs) {
    }

    public default void setLeaderVoltage(double voltage) {
    }

    public default void resetLeaderEncoderPosition() {
    }
}
