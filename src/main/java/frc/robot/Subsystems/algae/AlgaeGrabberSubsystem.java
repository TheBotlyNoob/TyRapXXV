package frc.robot.Subsystems.algae;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import frc.robot.Subsystems.algae.AlgaePneumaticsIO.AlgaeGrabberState;
import frc.robot.Utils.SafeableSubsystem;
import org.littletonrobotics.junction.Logger;

public class AlgaeGrabberSubsystem extends SafeableSubsystem {
  private final SparkMax retrieval_motor;

  private final AlgaePneumaticsIO pneumaticsIo;
  private final AlgaePneumaticsIOInputsAutoLogged pneumaticsInputs =
      new AlgaePneumaticsIOInputsAutoLogged();

  /** Creates a new AlgaeGrabber. */
  public AlgaeGrabberSubsystem(AlgaePneumaticsIO pneumaticsIO) {
    this.pneumaticsIo = pneumaticsIO;

    retrieval_motor = new SparkMax(Constants.MechID.kAlgaeMotorCanId, MotorType.kBrushless);
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.inverted(false);
    retrieval_motor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void extendGrabber() {
    pneumaticsIo.setState(AlgaeGrabberState.EXTENDED);
    retrieval_motor.set(-Constants.AlgaeGrabber.kIntakeSpeed);
  }

  public void retractGrabber() {
    pneumaticsIo.setState(AlgaeGrabberState.RETRACTED);
    retrieval_motor.set(0.0);
  }

  public void stopMotor() {
    retrieval_motor.set(0);
  }

  public void ejectAlgae() {
    retrieval_motor.set(Constants.AlgaeGrabber.kEjectSpeed);
  }

  public void makeSafe() {
    stopMotor();
    retractGrabber();
  }

  @Override
  public void periodic() {
    pneumaticsIo.updateInputs(pneumaticsInputs);
    Logger.processInputs("AlgaeGrabberSubsystem/Pneumatics", pneumaticsInputs);
  }
}
