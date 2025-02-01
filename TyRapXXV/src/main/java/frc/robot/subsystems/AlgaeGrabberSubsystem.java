package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

public class AlgaeGrabberSubsystem extends SubsystemBase {
  private SparkMax retrieval_motor;
  private DoubleSolenoid raise_pneumatics_solenoid;

  private ShuffleboardTab shuffle_tab = Shuffleboard.getTab(getName());

  private GenericEntry shuffle_motor_current = shuffle_tab.add("Motor Current", 0.0)
      .withWidget(BuiltInWidgets.kGraph)
      .withProperties(Map.of("max", 50.0)).withSize(4, 2)
      .withPosition(2, 0).getEntry();

  private GenericEntry shuffle_motor_voltage = shuffle_tab.add("Motor Voltage", 0.0)
      .withWidget(BuiltInWidgets.kNumberBar)
      .withProperties(Map.of("max", 12.0)).withSize(2, 1)
      .withPosition(0, 0).getEntry();

  private GenericEntry shuffle_motor_under_load = shuffle_tab.add("Motor Under Load", false)
      .withWidget(BuiltInWidgets.kBooleanBox).withSize(2, 1)
      .withPosition(0, 1).getEntry();

  /*
   * private GenericEntry shuffle_compressor_current =
   * shuffle_tab.add("Compressor Current Draw", 0.0)
   * .withWidget(BuiltInWidgets.kNumberBar)
   * .withSize(2, 1).withPosition(2, 0).getEntry();
   * private GenericEntry shuffle_compressor_pressure =
   * shuffle_tab.add("Compressor Pressure", 0.0)
   * .withWidget(BuiltInWidgets.kNumberBar)
   * .withSize(2, 1).withPosition(2, 1).getEntry();
   */

  private GenericEntry shuffle_solenoid_state = shuffle_tab.add("Solenoid State", "off")
      .withWidget(BuiltInWidgets.kTextView)
      .withSize(2, 1).withPosition(1, 2).getEntry();

  /** Creates a new AlgaeGrabber. */
  public AlgaeGrabberSubsystem() {
    retrieval_motor = new SparkMax(Constants.ID.kAlgaeGrabberMotorCANID, MotorType.kBrushless);

    // FIXME: put in proper solneoid type
    raise_pneumatics_solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
        Constants.ID.kAlgaeGrabberSolenoidCANID1,
        Constants.ID.kAlgaeGrabberSolenoidCANID2);
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
          retrieval_motor.set(0.1);
          // raise_pneumatics_solenoid.set(Value.kForward);

          new Trigger(() -> isMotorUnderLoad()).onTrue(runOnce(() -> {
            retrieval_motor.set(0.0);
            // raise_pneumatics_solenoid.set(Value.kOff);
          }));
        });
  }

  public boolean isMotorUnderLoad() {
    return false;
    // return retrieval_motor.getOutputCurrent() > 40;
  }

  @Override
  public void periodic() {
    shuffle_motor_current.setDouble(retrieval_motor.getOutputCurrent());
    shuffle_motor_voltage.setDouble(retrieval_motor.getAppliedOutput());

    shuffle_motor_under_load.setBoolean(isMotorUnderLoad());

    // shuffle_compressor_current.setDouble(raise_pneumatics_compressor.getCurrent());
    // shuffle_compressor_pressure.setDouble(raise_pneumatics_compressor.getPressure());

    // Value state = raise_pneumatics_solenoid.get();
    // shuffle_solenoid_state.setString(state == Value.kForward ? "forward" : state
    // == Value.kReverse ? "reverse" : "off");

    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
