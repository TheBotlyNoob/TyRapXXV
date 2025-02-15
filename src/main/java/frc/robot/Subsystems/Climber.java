package frc.robot.Subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climber extends SubsystemBase {
    private final SparkMax m_climbingMotor;

    public Climber(){
        m_climbingMotor = new SparkMax(Constants.MechID.kClimberCanId, MotorType.kBrushless);
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
    }
}
