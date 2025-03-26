package frc.robot.Subsystems.algae;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;

public class AlgaeRetrievalIOSpark implements AlgaeRetrievalIO {
    private final SparkMax m_motor = new SparkMax(Constants.MechID.kAlgaeMotorCanId, SparkLowLevel.MotorType.kBrushless);

    @Override
    public void updateInputs(AlgaeRetrievalIOInputs inputs) {
    }

    @Override
    public void setSpeed(double speed) {
        m_motor.set(speed);
    }

    @Override
    public void setVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }
}
