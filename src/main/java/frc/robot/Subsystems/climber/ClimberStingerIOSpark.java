package frc.robot.Subsystems.climber;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class ClimberStingerIOSpark implements ClimberStingerIO {
    private final SparkMax m_motor = new SparkMax(Constants.MechID.kClimberCanId, SparkLowLevel.MotorType.kBrushless);
    // TODO: move this out into a constant
    private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(9);


    @Override
    public void updateInputs(ClimberStingerIOInputs inputs) {
        inputs.dutyCycleEncoderPosition = Units.Rotations.of(m_encoder.get());
    }

    @Override
    public void setVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }
}
