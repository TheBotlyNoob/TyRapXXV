package frc.robot.Subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.PowerUnit;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        /**
         * The linear velocity of the drive motor.
         */
        public LinearVelocity driveMotorVelocity = Units.MetersPerSecond.of(0.0);

        public AngularVelocity turningMotorAngularVelocity = Units.RotationsPerSecond.of(0.0);
        public AngularVelocity driveMotorAngularVelocity = Units.RotationsPerSecond.of(0.0);

        /**
         * The orientation of wheel, accounting for encoder offsets. 0 is when
         * aligned with forward axis of the chasis.
         */
        public Angle turningMotorPosition = Units.Radians.of(0.0);

        /**
         * The orientation of the drive motor, not accounting for encoder offsets.
         */
        public Angle drivingMotorPosition = Units.Radians.of(0.0);

        /**
         * The distance the wheel thinks it has gone accross the floor.
         */
        public Distance drivingMotorDistance = Units.Meters.of(0.0);
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(SwerveModuleIOInputs inputs) {
    }

    // TODO: we can change this to use the Spark Max/Flex's built-in PID control,
    // saving latency slightly. Not necessary, though.

    /** Run the drive motor at the specified voltage. */
    public default void setDriveVoltage(Voltage voltage) {
    }

    /** Run the turn motor at the specified voltage. */
    public default void setTurnVoltage(Voltage voltage) {
    }

}
