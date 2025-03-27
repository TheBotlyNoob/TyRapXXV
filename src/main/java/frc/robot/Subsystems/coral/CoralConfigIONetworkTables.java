package frc.robot.Subsystems.coral;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import frc.robot.Constants;

public class CoralConfigIONetworkTables implements CoralConfigIO {
    private final DoubleEntry wristMotorVoltageForward;
    private final DoubleEntry wristMotorVoltageReverse;

    public CoralConfigIONetworkTables(NetworkTableInstance nt) {
        NetworkTable table = nt.getTable("Tuning").getSubTable("CoralSubsystemConfig");

        wristMotorVoltageForward = table.getDoubleTopic("wristMotorVoltageForward")
                .getEntry(Constants.Coral.kWristMotorVoltage);
        wristMotorVoltageForward.set(wristMotorVoltageForward.get());

        wristMotorVoltageReverse = table.getDoubleTopic("wristMotorVoltageReverse")
                .getEntry(Constants.Coral.kWristMotorVoltageReverse);
        wristMotorVoltageReverse.set(wristMotorVoltageReverse.get());
    }

    @Override
    public void updateInputs(CoralConfigIOInputs inputs) {
        inputs.wristMotorVoltageForward = Units.Volts.of(wristMotorVoltageForward.get());
        inputs.wristMotorVoltageReverse = Units.Volts.of(wristMotorVoltageReverse.get());
    }
}
