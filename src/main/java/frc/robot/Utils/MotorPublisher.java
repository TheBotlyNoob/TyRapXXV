package frc.robot.Utils;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;

/**
 * Publishes motor data to a NetworkTable under the specified subtable.
 */
public class MotorPublisher {
    protected final SparkBase motor;
    protected final RelativeEncoder encoder;

    private final NetworkTable table;

    private final DoublePublisher table_motor_current;
    private final DoublePublisher table_motor_voltage;
    private final DoublePublisher table_motor_position;
    private final DoublePublisher table_motor_velocity;

    public MotorPublisher(SparkBase motor, NetworkTable table, String name) {
        this.motor = motor;
        this.encoder = motor.getEncoder();

        this.table = table;

        table_motor_current = table.getSubTable(name).getDoubleTopic("current").publish();
        table_motor_voltage = table.getSubTable(name).getDoubleTopic("voltage").publish();
        table_motor_position = table.getSubTable(name).getDoubleTopic("position").publish();
        table_motor_velocity = table.getSubTable(name).getDoubleTopic("velocity").publish();
    }

    public void update() {
        table_motor_current.set(motor.getOutputCurrent());
        table_motor_position.set(encoder.getPosition());
        table_motor_velocity.set(encoder.getVelocity()/60.0); // Divide by 60 to get rotations per second
        table_motor_voltage.set(motor.getBusVoltage()* motor.getAppliedOutput());
    }
}
