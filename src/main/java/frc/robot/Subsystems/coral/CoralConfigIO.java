package frc.robot.Subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public interface CoralConfigIO {
    @AutoLog
    public static class CoralConfigIOInputs {
        public Voltage wristMotorVoltageForward = Units.Volts.of(Constants.Coral.kWristMotorVoltage);
        public Voltage wristMotorVoltageReverse = Units.Volts.of(Constants.Coral.kWristMotorVoltageReverse);
    }

    public default void updateInputs(CoralConfigIOInputs inputs) {

    }
}
