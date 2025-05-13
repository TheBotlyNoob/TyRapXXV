package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CoralSubsystem;

public class EjectCoral extends Command {
  CoralSubsystem co;

  public EjectCoral(CoralSubsystem co) {
    this.co = co;
    addRequirements(co);
  }

  @Override
  public void initialize() {
    System.out.println("Eject coral");
    co.ejectCoral();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
