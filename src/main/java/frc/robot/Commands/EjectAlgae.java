package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.AlgaeGrabberSubsystem;

public class EjectAlgae extends Command {
  AlgaeGrabberSubsystem ag;

  public EjectAlgae(AlgaeGrabberSubsystem ag) {
    this.ag = ag;
    addRequirements(ag);
  }

  @Override
  public void execute() {
    ag.ejectAlgae();
  }

  @Override
  public void end(boolean interupted) {
    ag.stopMotor();
  }
}
