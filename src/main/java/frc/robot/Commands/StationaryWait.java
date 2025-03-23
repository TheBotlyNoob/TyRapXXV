package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.drive.Drivetrain;

public class StationaryWait extends Command {

    protected Drivetrain dt;
    protected final Timer timer = new Timer();
    protected double durationSec;

    public StationaryWait(Drivetrain dt, double durationSec) {
        this.dt = dt;
        this.durationSec = durationSec;

        addRequirements(dt);
    }

    public void execute() {
        this.dt.driveChassisSpeeds(new ChassisSpeeds());
    }

    public boolean isFinished() {
        return (timer.get() >= durationSec);
    }

}
