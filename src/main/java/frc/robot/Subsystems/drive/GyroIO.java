package frc.robot.Subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public Rotation2d yawPosition = new Rotation2d(Units.Radians.of(0.0));
        public AngularVelocity yawVelocity = Units.RadiansPerSecond.of(0.0);
    }

    public default void updateInputs(GyroIOInputs inputs) {
    }

    /**
     * Sets the yaw of the gyro.
     *
     * @returns whether the call succeeded
     */
    public default boolean setYaw(Rotation2d yaw) {
        return false;
    }
}
