package frc.robot.Subsystems.algae;

import frc.robot.Constants;
import frc.robot.Subsystems.algae.AlgaePneumaticsIO.AlgaeGrabberState;
import frc.robot.Utils.SafeableSubsystem;
import org.littletonrobotics.junction.Logger;

public class AlgaeGrabberSubsystem extends SafeableSubsystem {
  private final AlgaeRetrievalIO retrievalIo;
  private final AlgaeRetrievalIOInputsAutoLogged retrievalInputs =
      new AlgaeRetrievalIOInputsAutoLogged();

  private final AlgaePneumaticsIO pneumaticsIo;
  private final AlgaePneumaticsIOInputsAutoLogged pneumaticsInputs =
      new AlgaePneumaticsIOInputsAutoLogged();

  /** Creates a new AlgaeGrabber. */
  public AlgaeGrabberSubsystem(AlgaePneumaticsIO pneumaticsIo, AlgaeRetrievalIO retrievalIo) {
    this.pneumaticsIo = pneumaticsIo;
    this.retrievalIo = retrievalIo;
  }

  public void extendGrabber() {
    pneumaticsIo.setState(AlgaeGrabberState.EXTENDED);
    retrievalIo.setSpeed(-Constants.AlgaeGrabber.kIntakeSpeed);
  }

  public void retractGrabber() {
    pneumaticsIo.setState(AlgaeGrabberState.RETRACTED);
    stopMotor();
  }

  public void stopMotor() {
    retrievalIo.setSpeed(0);
  }

  public void ejectAlgae() {
    retrievalIo.setSpeed(Constants.AlgaeGrabber.kEjectSpeed);
  }

  public void makeSafe() {
    stopMotor();
    retractGrabber();
  }

  @Override
  public void periodic() {
    pneumaticsIo.updateInputs(pneumaticsInputs);
    Logger.processInputs("AlgaeGrabberSubsystem/Pneumatics", pneumaticsInputs);

    retrievalIo.updateInputs(retrievalInputs);
    Logger.processInputs("AlgaeGrabberSubsystem/Retrieval", retrievalInputs);
  }
}
