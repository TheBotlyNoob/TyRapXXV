package frc.robot.Subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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
    protected final AbsoluteEncoder m_encoder;
    protected final MotorPublisher m_climbMotorPublisher;

    protected final DoubleSolenoid m_clampPneumatic;
    protected final DoubleSolenoid m_lowerPneumatic;

    public ClimberSubsystem(NetworkTableInstance nt) {
        table = nt.getTable(getName());
        m_climbingMotor = new SparkMax(Constants.MechID.kClimberCanId, MotorType.kBrushless);
        m_climbMotorPublisher = new MotorPublisher(m_climbingMotor, table, "climbingMotor");

        m_encoder = m_climbingMotor.getAbsoluteEncoder();

        m_clampPneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Climber.kClampSolenoidCANID1, Constants.Climber.kClampSolenoidCANID2);
        m_lowerPneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Climber.kLowerSolenoidCANID1, Constants.Climber.kLowerSolenoidCANID2);
    }

    public void extendArms() {
        m_lowerPneumatic.set(Value.kForward);
    }

    public void moveArmsIn(){
        if(m_lowerPneumatic.get() == Value.kForward) {
            m_clampPneumatic.set(Value.kForward);
        }
    }

    public void moveArmsOut() {
        m_clampPneumatic.set(Value.kReverse);
    }
    
    public void resetArms() {
        // TODO: unclamp before raising
        m_clampPneumatic.set(Value.kReverse);
        m_lowerPneumatic.set(Value.kReverse);
    }

    private void reverseMotor(){
        m_climbingMotor.setVoltage(-Constants.Climber.kClimbMotorVoltage);
    }
    
    private void forwardMotor(){
        m_climbingMotor.setVoltage(Constants.Climber.kClimbMotorVoltage);
    }

    public void stopMotor(){
        m_climbingMotor.setVoltage(0.0);
    }

    public void extendStinger() {
        if(m_clampPneumatic.get() == Value.kForward && m_lowerPneumatic.get() == Value.kForward) {
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
