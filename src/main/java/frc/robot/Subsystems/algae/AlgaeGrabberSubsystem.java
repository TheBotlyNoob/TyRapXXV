package frc.robot.Subsystems.algae;

import frc.robot.Constants;
import frc.robot.Utils.SafeableSubsystem;
import org.littletonrobotics.junction.Logger;

public class AlgaeGrabberSubsystem extends SafeableSubsystem {
    private final AlgaeRetrievalIO m_retrievalIo;
    private final AlgaeRetrievalIOInputsAutoLogged m_retrievalInputs = new AlgaeRetrievalIOInputsAutoLogged();

    private final AlgaePneumaticsIO m_extenderIo;
    private final AlgaePneumaticsIOInputsAutoLogged m_extenderInputs = new AlgaePneumaticsIOInputsAutoLogged();

    /** Creates a new AlgaeGrabber. */
    public AlgaeGrabberSubsystem(AlgaePneumaticsIO extenderIo, AlgaeRetrievalIO retrievalIo) {
        m_extenderIo = extenderIo;
        m_retrievalIo = retrievalIo;
    }

    public void extendGrabber() {
        m_extenderIo.setState(AlgaePneumaticsIO.AlgaeGrabberState.EXTENDED);
        m_retrievalIo.setSpeed(-Constants.AlgaeGrabber.kIntakeSpeed);
    }

    public void retractGrabber() {
        m_extenderIo.setState(AlgaePneumaticsIO.AlgaeGrabberState.RETRACTED);
        m_retrievalIo.setSpeed(0.0);
    }

    public void stopMotor() {
        m_retrievalIo.setSpeed(0);
    }

    public void ejectAlgae() {
        m_retrievalIo.setSpeed(Constants.AlgaeGrabber.kEjectSpeed);
    }

    public void makeSafe() {
        stopMotor();
        retractGrabber();
    }

    @Override
    public void periodic() {
        m_extenderIo.updateInputs(m_extenderInputs);
        Logger.processInputs("AlgaeGrabberSubsystem/Pneumatics", m_extenderInputs);

        m_retrievalIo.updateInputs(m_retrievalInputs);
        Logger.processInputs("AlgaeGrabberSubsystem/Retrieval", m_retrievalInputs);
    }
}
