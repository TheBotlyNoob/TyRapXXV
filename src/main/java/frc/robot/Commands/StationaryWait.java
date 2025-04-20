package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;

public class StationaryWait extends Command {

  protected Drivetrain dt;
  protected double durationSec;
  protected double endTimeSec;

  public StationaryWait(Drivetrain dt, double durationSec) {
    this.dt = dt;
    this.durationSec = durationSec;
  }

  public void initialize() {
    this.endTimeSec = Timer.getFPGATimestamp() + durationSec;
  }

  public void execute() {
    this.dt.driveChassisSpeeds(new ChassisSpeeds());
  }

  public boolean isFinished() {
    return (Timer.getFPGATimestamp() >= endTimeSec);
  }
}
