package frc.robot.Commands;

import frc.robot.Subsystems.Limelight;

public class WaitForLLCommand {
  // Command to wait until limelight detects a valid
  // April tag. Can decorate with .withTimeout(timeout) to
  // prevent it from running forever if a tag is never detected
  protected final Limelight ll;

  public WaitForLLCommand(Limelight ll) {
    this.ll = ll;
  }

  public boolean isFinished() {
    return (ll.getTimeSinceValid() < 1);
  }
}
