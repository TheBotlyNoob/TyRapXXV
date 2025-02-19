package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils.MotorPublisher;

public class CoralSubsystem extends SubsystemBase {
    private final NetworkTable m_table;

    private final SparkMax m_containerMotor;
    private final SparkMax m_coralMotor;

    private boolean pointedOut = false;

    private final MotorPublisher m_containerMotorPublisher;
    private final MotorPublisher m_coralMotorPublisher;

    private final StringPublisher m_table_level;

    public CoralSubsystem(NetworkTableInstance nt) {
        m_table = nt.getTable(getName());

        m_table_level = m_table.getStringTopic("coral").publish();

        m_containerMotor = new SparkMax(Constants.MechID.kCoralCanId, MotorType.kBrushless);
        m_coralMotor = new SparkMax(Constants.MechID.kCoralCanId, MotorType.kBrushless);

        m_containerMotorPublisher = new MotorPublisher(m_containerMotor, m_table, "container");
        m_coralMotorPublisher = new MotorPublisher(m_coralMotor, m_table, "coral");

    }

    public void pointOut() {
        // TODO: voltage
        m_containerMotor.set(0.0);
        // TODO: maybe move this to a trigger?
        pointedOut = true;
    }

    public void retractContainer() {
        // TODO: voltage
        m_containerMotor.set(-0.0);
        pointedOut = false;
    }

    public void pushCoral() {
        if (pointedOut) {
            // TODO: voltage
            m_coralMotor.set(0.0);
            // TODO: trigger to stop after a bit
        }
    }

    public void setVoltageTest(double voltage) {
        System.out.println("Setting Coral voltage " + voltage);
        m_containerMotor.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        m_containerMotorPublisher.update();
        m_coralMotorPublisher.update();
    }
}
