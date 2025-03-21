package frc.robot.Subsystems.coral;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import frc.robot.Constants;

public class CoralConfigIONetworkTables implements CoralConfigIO {
    private final DoubleSubscriber wristMotorVoltageForward;
    private final DoubleSubscriber wristMotorVoltageReverse;

    public CoralConfigIONetworkTables(NetworkTableInstance nt) {
        NetworkTable table = nt.getTable("CoralSubsystemConfig");
        wristMotorVoltageForward = table.getDoubleTopic("wristMotorVoltageForward")
                .subscribe(Constants.Coral.kWristMotorVoltage);
        wristMotorVoltageReverse = table.getDoubleTopic("wristMotorVoltageReverse")
                .subscribe(Constants.Coral.kWristMotorVoltageReverse);
    }

    @Override
    public void updateInputs(CoralConfigIOInputs inputs) {
        inputs.wristMotorVoltageForward = Units.Volts.of(wristMotorVoltageForward.get());
        inputs.wristMotorVoltageReverse = Units.Volts.of(wristMotorVoltageReverse.get());
    }
}
