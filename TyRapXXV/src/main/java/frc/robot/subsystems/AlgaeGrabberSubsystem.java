package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class AlgaeGrabberSubsystem extends SubsystemBase {
  private final SparkMax retrieval_motor;
  private final DoubleSolenoid raise_pneumatics_solenoid;

  private final NetworkTable table;

  private final DoublePublisher table_motor_current;
  private final DoublePublisher table_motor_voltage;

  private final BooleanPublisher table_motor_under_load;

  private final StringPublisher table_solenoid_state;

  /** Creates a new AlgaeGrabber. */
  public AlgaeGrabberSubsystem(NetworkTableInstance nt) {
    retrieval_motor = new SparkMax(Constants.ID.kAlgaeGrabberMotorCANID, MotorType.kBrushless);

    // FIXME: put in proper solneoid type
    raise_pneumatics_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
        Constants.ID.kAlgaeGrabberSolenoidCANID1,
        Constants.ID.kAlgaeGrabberSolenoidCANID2);

    table = nt.getTable(getName());

    table_motor_current = table.getDoubleTopic("motor_current").publish();
    table_motor_voltage = table.getDoubleTopic("motor_voltage").publish();

    table_motor_under_load = table.getBooleanTopic("motor_under_load").publish();

    table_solenoid_state = table.getStringTopic("solenoid_state").publish();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command beginRetrieval() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          retrieval_motor.set(0.2);
          raise_pneumatics_solenoid.set(Value.kForward);

          new Trigger(() -> isMotorUnderLoad()).onTrue(runOnce(() -> {
            retrieval_motor.set(0.0);
            raise_pneumatics_solenoid.set(Value.kReverse);
          }));
        });
  }

  public boolean isMotorUnderLoad() {
    return retrieval_motor.getOutputCurrent() > 21;
  }

  @Override
  public void periodic() {
    table_motor_current.set(retrieval_motor.getOutputCurrent());
    table_motor_voltage.set(retrieval_motor.getAppliedOutput());

    table_motor_under_load.set(isMotorUnderLoad());

    Value state = raise_pneumatics_solenoid.get();
    table_solenoid_state.set(state == Value.kForward ? "forward" : state == Value.kReverse ? "reverse" : "off");

    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
