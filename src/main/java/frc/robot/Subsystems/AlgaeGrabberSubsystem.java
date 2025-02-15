package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.BooleanEntry;
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
import frc.robot.Utils.Elastic;
import frc.robot.Utils.Elastic.Notification;
import frc.robot.Utils.Elastic.Notification.NotificationLevel;
import frc.robot.Utils.MotorPublisher;

public class AlgaeGrabberSubsystem extends SubsystemBase {
  private final Value pneumaticOut = Value.kReverse;
  private final Value pneumaticIn = Value.kForward;
  private final Value pneumaticOff = Value.kOff;

  private final SparkMax retrieval_motor;
  private final DoubleSolenoid raise_pneumatics_solenoid;

  private final NetworkTable table;

  private final MotorPublisher motorPublisher;

  private final BooleanPublisher table_motor_stalled;

  private final StringPublisher table_solenoid_state;

  /**
   * Should the robot extend the algae retriever and run the motor?
   */
  private final BooleanEntry table_should_retrieve;

  /** Creates a new AlgaeGrabber. */
  public AlgaeGrabberSubsystem(NetworkTableInstance nt) {
    retrieval_motor = new SparkMax(Constants.MechID.kAlgaeMotorCanId, MotorType.kBrushless);

    raise_pneumatics_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
        Constants.ID.kAlgaeGrabberSolenoidCANID1,
        Constants.ID.kAlgaeGrabberSolenoidCANID2);

    table = nt.getTable(getName());

    motorPublisher = new MotorPublisher(retrieval_motor, table, "motor");

    table_motor_stalled = table.getBooleanTopic("motor_stalled").publish();

    table_solenoid_state = table.getStringTopic("solenoid_state").publish();

    table_should_retrieve = table.getBooleanTopic("should_retrieve").getEntry(false);
    // needed to make the entry show up in elastic
    table_should_retrieve.set(false);

    new Trigger(() -> table_should_retrieve.get()).and(() -> !isMotorStalled()).onTrue(runOnce(() -> {
      retrieval_motor.set(Constants.AlgaeGrabber.kMotorSpeed);
      raise_pneumatics_solenoid.set(pneumaticOut);
    })).onFalse(runOnce(() -> {
      table_should_retrieve.set(false);
      retrieval_motor.set(0.0);
      raise_pneumatics_solenoid.set(pneumaticIn);
    }));
  }

  public Command toggleRetriever() {
    return runOnce(() -> {
      Elastic.sendNotification(new Notification(NotificationLevel.INFO, getName(), "toggling algae retrieval"));
      table_should_retrieve.set(!table_should_retrieve.get());
    });
  }

  public boolean isMotorStalled() {
    return retrieval_motor.getOutputCurrent() > Constants.AlgaeGrabber.kMotorCurrentThreshold;
  }

  @Override
  public void periodic() {
    motorPublisher.update();

    table_motor_stalled.set(isMotorStalled());

    Value state = raise_pneumatics_solenoid.get();
    table_solenoid_state.set(state == pneumaticOut ? "out" : state == pneumaticIn ? "in" : "off");

    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
