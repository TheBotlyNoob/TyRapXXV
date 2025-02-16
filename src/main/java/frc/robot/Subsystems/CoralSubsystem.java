package frc.robot.Subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils.MotorPublisher;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

public class CoralSubsystem extends SubsystemBase {


    private final NetworkTable m_table;
    private final WPI_TalonSRX m_motorCoral;
    //private final MotorPublisher m_motorPublisherCoral;
    private final StringPublisher m_table_level;

    public CoralSubsystem(NetworkTableInstance nt) {
        m_table = nt.getTable(getName());
        m_table_level = m_table.getStringTopic("coral").publish();


        m_motorCoral = new WPI_TalonSRX(Constants.MechID.kCoralCanId);

        TalonSRXConfiguration motorConf = new TalonSRXConfiguration();
        m_motorCoral.getAllConfigs(motorConf);
        //Configure the motor
        m_motorCoral.configAllSettings(motorConf);
    
        //m_motorPublisherCoral = new MotorPublisher(m_motorCoral, m_table, "coral");

    }

    public void setVoltageTest(double voltage) {
        System.out.println("Setting Coral voltage " + voltage);
        m_motorCoral.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        //m_motorPublisherCoral.update();
        //m_motorPublisherCoral.update();
    }
}
