package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.algae.AlgaeGrabberSubsystem;

public class AlgaeIntake extends Command {
  AlgaeGrabberSubsystem ag;

  public AlgaeIntake(AlgaeGrabberSubsystem ag) {
    this.ag = ag;
    addRequirements(ag);
  }

  @Override
  public void execute() {
    ag.extendGrabber();
  }

  @Override
  public void end(boolean interupted) {
    ag.retractGrabber();
  }
}
