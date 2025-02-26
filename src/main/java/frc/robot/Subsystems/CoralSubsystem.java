package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils.MotorPublisher;

public class CoralSubsystem extends SubsystemBase {
    /*
    TODO: Fix limit switch logic 
    Commands:
     * eject coral 
     * run little motor until line sensor (default)
     * fully extended econder position fully retracted encoder position
     */
    private final NetworkTable m_table;

    private final SparkMax m_wristMotor;
    private final SparkMax m_coralGrabberMotor;

    private boolean pointedOut = false;
    private double holdPosition = 0.0;

    private final MotorPublisher m_wristMotorPublisher;
    private final MotorPublisher m_coralGrabberMotorPublisher;

    private final StringPublisher m_table_level;

    protected final AbsoluteEncoder m_wristEncoder;
    private final DoublePublisher m_wristEncoderPub;

    public CoralSubsystem(NetworkTableInstance nt) {
        m_table = nt.getTable(getName());

        m_table_level = m_table.getStringTopic("coral").publish();

        m_coralGrabberMotor = new SparkMax(Constants.MechID.kCoralWheelCanId, MotorType.kBrushless);
        m_wristMotor = new SparkMax(Constants.MechID.kCoralWristCanId, MotorType.kBrushed);

        m_coralGrabberMotorPublisher = new MotorPublisher(m_coralGrabberMotor, m_table, "Grabber Motor");
        m_wristMotorPublisher = new MotorPublisher(m_wristMotor, m_table, "Wrist Motor");
        
        m_wristEncoder = m_wristMotor.getAbsoluteEncoder();
        m_wristEncoderPub = nt.getDoubleTopic("Wrist Encoder").publish();
    }

    public void ejectCoral() {
        if (pointedOut) {
            // TODO: voltage
            m_coralGrabberMotor.set(0.0);
            // TODO: trigger to stop after a bit
        }
    }

    public void setVoltageTest(double voltage) {
        System.out.println("Setting Coral voltage " + voltage);
        m_wristMotor.setVoltage(voltage);
    }

    public void reverseMotor(){
        // if (m_wristEncoder.getPosition() <= Constants.Coral.kMinEncoderPos){
        //     stopMotor();
        // }
        // else {
            m_wristMotor.setVoltage(-Constants.Coral.kWristMotorVoltage);
       // }
    }
        
    public void forwardMotor(){
        // if (m_wristEncoder.getPosition() >= Constants.Coral.kMaxEncoderPos){
        //     stopMotor();
        // }
        // else {
            m_wristMotor.setVoltage(Constants.Coral.kWristMotorVoltage);
        //}
    }
        
    public void stopMotor(){
        m_wristMotor.setVoltage(0.0);
        holdPosition = m_wristEncoder.getPosition();

    }
        
    public void extendManipulator() {
        forwardMotor();
    }
        
        
    public void retractManipulator() {
        reverseMotor();
    }

    @Override
    public void periodic() {
        m_wristMotorPublisher.update();
        m_coralGrabberMotorPublisher.update();
//        m_wristMotorPublisher.update();
        m_wristEncoderPub.set(m_wristEncoder.getPosition());
    }
}

