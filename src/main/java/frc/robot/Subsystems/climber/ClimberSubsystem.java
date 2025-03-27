package frc.robot.Subsystems.climber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils.MotorPublisher;
import frc.robot.Utils.SafeableSubsystem;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

// this doesn't make sense as a `SafeableSubsystem`
public class ClimberSubsystem extends SubsystemBase {

    protected final NetworkTable table;
    protected final DutyCycleEncoder m_climbEncoder;
    private final DoublePublisher m_encoderPub;
    protected boolean isClimbMode;

    protected SafeableSubsystem[] m_toMakeSafe;

    protected final SparkMax m_climbingMotor;
    protected final MotorPublisher m_climbMotorPublisher;
    protected final DoublePublisher m_encoderPublisher;

    protected final DoubleSolenoid m_clampPneumatic;
    protected final DoubleSolenoid m_lowerPneumatic;
    protected final DoubleSolenoid m_rampPneumatic;

    private final Value kArmsExtend = Value.kForward; // grabber arms extends and lower to start climb
    private final Value kArmsRetract = Value.kReverse;

    private final Value kGrabberClose = Value.kForward; // grabber clamps to cage
    private final Value kGrabberOpen = Value.kReverse;

    private final Value kRampUp = Value.kForward; //
    private final Value kRampDown = Value.kReverse;; // unclamps

    public ClimberSubsystem(AbsoluteEncoder climbArmEncoder, NetworkTableInstance nt, SafeableSubsystem[] toMakeSafe) {
        table = nt.getTable(getName());
        m_climbingMotor = new SparkMax(Constants.MechID.kClimberCanId, MotorType.kBrushless);
        m_climbMotorPublisher = new MotorPublisher(m_climbingMotor, table, "climbingMotor");
        m_encoderPublisher = table.getDoubleTopic("absolute encoder").publish();
        // m_climbEncoder = climbArmEncoder;
        m_climbEncoder = new DutyCycleEncoder(9);
        m_encoderPub = nt.getDoubleTopic("Encoder Position").publish();
        isClimbMode = false;

        m_clampPneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Climber.kClampSolenoidCANID1,
                Constants.Climber.kClampSolenoidCANID2);
        m_lowerPneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Climber.kLowerSolenoidCANID1,
                Constants.Climber.kLowerSolenoidCANID2);
        m_rampPneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Climber.kRampSolenoidCANID1,
                Constants.Climber.kRampSolenoidCANID2);

        m_lowerPneumatic.set(kArmsRetract);
        m_clampPneumatic.set(kGrabberOpen);
        m_rampPneumatic.set(kRampDown);

        m_toMakeSafe = toMakeSafe;
    }

    public boolean isClimbMode() {
        return isClimbMode;
    }

    public boolean isCoralMode() {
        return !isClimbMode;
    }

    public void toggleClimbMode() {
        System.out.println("Toggling climb current=" + isClimbMode);
        if (isClimbMode) {
            setCoralMode();
        } else {
            setClimbMode();
        }
    }

    public void setClimbMode() {
        extendArms();
        rampUp();
        m_clampPneumatic.set(kGrabberOpen);
        for (int i = 0; i < m_toMakeSafe.length; i++) {
            System.out.println("Placing " + m_toMakeSafe[i].getName() + " into a safe position");
            m_toMakeSafe[i].makeSafe();
        }
        isClimbMode = true;
    }

    public void setCoralMode() {
        retractArms();
        rampDown();
        m_clampPneumatic.set(kGrabberOpen);
        isClimbMode = false;
    }

    public void toggleGrabArms() {
        System.out.println("Toggling grab arms current=" + m_clampPneumatic.get());
        m_clampPneumatic.toggle();
    }

    public void extendArms() {
        m_lowerPneumatic.set(kArmsExtend);
    }

    public void retractArms() {
        m_lowerPneumatic.set(kArmsRetract);
    }

    public void toggleRamp() {
        System.out.println("Toggling ramp current=" + m_rampPneumatic.get());
        m_rampPneumatic.toggle();
    }

    public void rampUp() {
        m_rampPneumatic.set(kRampUp);
    }

    public void rampDown() {
        m_rampPneumatic.set(kRampDown);
    }

    public void reverseMotor() {
        System.out.println("reverseMotor called");
        if (m_climbEncoder.get() <= Constants.Climber.kMinEncoderPos) {
            stopMotor();
        } else {
            m_climbingMotor.setVoltage(-Constants.Climber.kClimbMotorVoltage);
        }
    }

    public void forwardMotor() {
        System.out.println("forwardMotor called");

        if (m_climbEncoder.get() >= Constants.Climber.kMaxEncoderPos) {
            stopMotor();
        } else {
            m_climbingMotor.setVoltage(Constants.Climber.kClimbMotorVoltage);
        }
    }

    public void stopMotor() {
        m_climbingMotor.setVoltage(0.0);
    }

    public void extendStinger() {
        forwardMotor();
    }

    public void retractStinger() {
        reverseMotor();
    }

    @Override
    public void periodic() {
        m_climbMotorPublisher.update();
        m_encoderPub.set(m_climbEncoder.get());
        m_encoderPublisher.set(m_climbEncoder.get());
    }
}
