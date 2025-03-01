package frc.robot.Subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Commands.EjectCoral;
import frc.robot.Utils.MotorPublisher;
import edu.wpi.first.networktables.DoubleEntry;

/*
 * 
 * different speeds up vs down 
 * IR sensor 
 * holde break logic
 */
public class CoralSubsystem extends SubsystemBase {
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

    private final DigitalInput m_irSensor;
    private final DoublePublisher m_irSensorPub;

    protected DoubleEntry kWristMotorVoltageForward;
    protected DoubleEntry kWristMotorVoltageReverse;
    protected DoubleEntry kWristMotorSpeedForward;
    protected DoubleEntry kWristMotorSpeedReverse;

    private double counter;
    private double start;
    private boolean ejectActive = false;
    private Timer timer;

    public enum CoralState {
        WAITING, INTAKING, HOLDING, EJECTING;
    };

    protected CoralState state = CoralState.WAITING;

    public CoralSubsystem(NetworkTableInstance nt) {
        m_table = nt.getTable(getName());

        m_table_level = m_table.getStringTopic("coral").publish();

        m_coralGrabberMotor = new SparkMax(Constants.MechID.kCoralWheelCanId, MotorType.kBrushless);
        m_wristMotor = new SparkMax(Constants.MechID.kCoralWristCanId, MotorType.kBrushed);

        kWristMotorVoltageForward = m_table.getDoubleTopic("wrist motor voltage forward").getEntry(Constants.Coral.kWristMotorVoltage);
        kWristMotorVoltageReverse = m_table.getDoubleTopic("wrist motor voltage reverse").getEntry(Constants.Coral.kWristMotorVoltageReverse);

        // kWristMotorSpeedForward = m_table.getDoubleTopic("wrist motor speed
        // forward").getEntry(0.0);
        // kWristMotorSpeedReverse = m_table.getDoubleTopic("wrist motor speed
        // reverse").getEntry(0.0);

        kWristMotorVoltageForward.set(Constants.Coral.kWristMotorVoltage);
        kWristMotorVoltageReverse.set(Constants.Coral.kWristMotorVoltageReverse);
        // kWristMotorSpeedForward.set(0.0);
        // kWristMotorSpeedReverse.set(0.0);

        m_coralGrabberMotorPublisher = new MotorPublisher(m_coralGrabberMotor, m_table, "Grabber Motor");
        m_wristMotorPublisher = new MotorPublisher(m_wristMotor, m_table, "Wrist Motor");

        m_wristEncoder = m_coralGrabberMotor.getAbsoluteEncoder();
        m_wristEncoderPub = nt.getDoubleTopic("Wrist Encoder").publish();

        m_irSensor = new DigitalInput(Constants.SensorID.kIRSensorPort);
        m_irSensorPub = nt.getDoubleTopic("IR Sensor").publish();

        timer = new Timer();
    }

    public void ejectCoral() {
        ejectActive = true;
        m_coralGrabberMotor.set(0.5);
    }

    // public void setVoltageTest(double voltage) {
    // System.out.println("Setting Coral voltage " + voltage);
    // m_wristMotor.setVoltage(voltage);
    // }

    public void reverseMotor() {
        double voltage = kWristMotorVoltageReverse.get();
        // double speed = kWristMotorSpeedReverse.get();
        // m_wristMotor.set(-speed);
        if (m_wristEncoder.getPosition() <= Constants.Coral.kMinEncoderPos) {
            stopMotorWrist();
        } else {
            m_wristMotor.setVoltage(voltage);
        }

    }

    public void forwardMotor() {
        double voltage = kWristMotorVoltageForward.get();
        // double speed = kWristMotorSpeedForward.get();
        if (m_wristEncoder.getPosition() >= Constants.Coral.kMaxEncoderPos) {
            stopMotorWrist();
        } else {
            m_wristMotor.setVoltage(-voltage);
        }

        // m_wristMotor.set(speed);
    }

    public void stopMotorWrist() {
        holdPosition = m_wristEncoder.getPosition();
        m_wristMotor.setVoltage(0.0);
        // m_wristMotor.set(0.0);
    }

    public void stopMotorGrabber() {
        m_coralGrabberMotor.set(0.0);
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
        m_wristEncoderPub.set(m_wristEncoder.getPosition());

        boolean irDetected = !m_irSensor.get();
        m_irSensorPub.set(irDetected ? 1.0 : 0.0);

        if (state == CoralState.WAITING) {
            if (irDetected) {
                state = CoralState.INTAKING;
                counter = m_coralGrabberMotor.getEncoder().getPosition();
                start = counter;
            } else {
                m_coralGrabberMotor.set(0.1);
            }
        } else if (state == CoralState.INTAKING) {
            m_coralGrabberMotor.set(0.1);
            counter = m_coralGrabberMotor.getEncoder().getPosition();
            if (counter >= (start + 7.0)) {
                state = CoralState.HOLDING;
            }
        } else if (state == CoralState.HOLDING) {
            m_coralGrabberMotor.set(0.0);
            if (ejectActive) {
                state = CoralState.EJECTING;
            }
        } else if (state == CoralState.EJECTING) {
            m_coralGrabberMotor.set(0.5);
            timer.start();
            if (timer.get() > 2 && !irDetected) {
                state = CoralState.WAITING;
                ejectActive = false;
            }
        }
    }
}
