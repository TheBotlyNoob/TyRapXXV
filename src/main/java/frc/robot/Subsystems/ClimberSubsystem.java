package frc.robot.Subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Utils.MotorPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


public class ClimberSubsystem extends SubsystemBase {
    
    protected final NetworkTable table;
    
    protected final SparkMax m_climbingMotor;
    protected final MotorPublisher m_climbMotorPublisher;
    protected final DoublePublisher m_encoderPublisher;

    protected final DoubleSolenoid m_clampPneumatic;
    protected final DoubleSolenoid m_lowerPneumatic;
    
    private final Value kArmsExtend = Value.kForward; //grabber arms extends and lower to start climb
    private final Value kArmsRetract = Value.kReverse; 
    
    private final Value kGrabberClose = Value.kForward; //grabber clamps to cage
    private final Value kGrabberOpen = Value.kReverse; //unclamps

    public ClimberSubsystem(NetworkTableInstance nt) {
        table = nt.getTable(getName());
        m_climbingMotor = new SparkMax(Constants.MechID.kClimberCanId, MotorType.kBrushless);
        m_climbMotorPublisher = new MotorPublisher(m_climbingMotor, table, "climbingMotor");
        m_encoderPublisher = table.getDoubleTopic("absolute encoder").publish();
        

        //m_encoder = m_climbingMotor.getAbsoluteEncoder();

        m_clampPneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Climber.kClampSolenoidCANID1, Constants.Climber.kClampSolenoidCANID2);
        m_lowerPneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Climber.kLowerSolenoidCANID1, Constants.Climber.kLowerSolenoidCANID2);
    }

    public void extendArms() {
        m_lowerPneumatic.set(kArmsExtend);
    }

    public void moveArmsIn(){
        //if(m_lowerPneumatic.get() == kArmsExtend) {
        m_clampPneumatic.set(kGrabberClose);
    }

    public void moveArmsOut() {
        m_clampPneumatic.set(kGrabberOpen);
    }
    
    public void retractArms() {
        // TODO: unclamp before raising
        //m_clampPneumatic.set(kGrabberOpen);
        m_lowerPneumatic.set(kArmsRetract);
    }

    public void reverseMotor(){
        //if (m_encoder.getPosition() <= Constants.Climber.kMinEncoderPos){
        //    stopMotor();
        //}
        m_climbingMotor.setVoltage(-Constants.Climber.kClimbMotorVoltage);
    }
    
    public void forwardMotor(){
        //if (m_encoder.getPosition() >= Constants.Climber.kMaxEncoderPos){
        //    stopMotor();
        //}
        m_climbingMotor.setVoltage(Constants.Climber.kClimbMotorVoltage);
    }

    public void stopMotor(){
        m_climbingMotor.setVoltage(0.0);
    }

    public void extendStinger() {
        if(m_clampPneumatic.get() == kGrabberClose && m_lowerPneumatic.get() == kArmsExtend) {
            forwardMotor();
        }
    }

    public void retractStinger() {
        reverseMotor();
    }
    
    @Override
    public void periodic() {
        m_climbMotorPublisher.update();
    }
}
