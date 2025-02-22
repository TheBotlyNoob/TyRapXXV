package frc.robot.Subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climber extends SubsystemBase {
    private final SparkMax m_climbingMotor;
    protected final AbsoluteEncoder m_climbEncoder;
    private final DoublePublisher m_encoderPub;

    public Climber(AbsoluteEncoder climbArmEncoder, NetworkTableInstance nt){
        m_climbingMotor = new SparkMax(Constants.MechID.kClimberCanId, MotorType.kBrushless);
        m_climbEncoder = climbArmEncoder;
        m_encoderPub = nt.getDoubleTopic("Encoder Position").publish();
        }

    public Command startMotor(){
        return runOnce(() -> {
        m_climbingMotor.setVoltage(0.5);
        });

    }
    public Command stopMotor(){
        return runOnce(() -> {
        m_climbingMotor.setVoltage(0.0);
        });
        
    }

    @Override
    public void periodic() {
        m_encoderPub.set(m_climbEncoder.getPosition());
    }
}
