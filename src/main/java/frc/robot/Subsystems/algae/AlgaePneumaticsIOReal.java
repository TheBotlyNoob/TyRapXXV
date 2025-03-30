package frc.robot.Subsystems.algae;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class AlgaePneumaticsIOReal implements AlgaePneumaticsIO {
    private static final DoubleSolenoid.Value kExtended = DoubleSolenoid.Value.kForward;
    private static final DoubleSolenoid.Value kRetracted = DoubleSolenoid.Value.kReverse;
    private static final DoubleSolenoid.Value kOff = DoubleSolenoid.Value.kOff;

    private final DoubleSolenoid m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.AlgaeGrabber.kSolenoidCANID1, Constants.AlgaeGrabber.kSolenoidCANID2);

    @Override
    public void updateInputs(AlgaePneumaticsIOInputs inputs) {
        inputs.state = m_solenoid.get() == kExtended
                ? AlgaeGrabberState.EXTENDED
                : m_solenoid.get() == kRetracted
                    ? AlgaeGrabberState.RETRACTED
                    : AlgaeGrabberState.OFF;
    }

    @Override
    public void setState(AlgaeGrabberState state) {
        switch (state) {
            case EXTENDED -> m_solenoid.set(kExtended);
            case RETRACTED -> m_solenoid.set(kRetracted);
            case OFF -> m_solenoid.set(kOff);
        }
    }
}
