package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;

public class ResetOdoCommand extends Command {
    Drivetrain dt;

    public ResetOdoCommand(Drivetrain dt) {
        this.dt = dt;
    }

    @Override
    public void schedule() {
        dt.resetGyro();
        dt.resetOdo();
    }

    @Override
    public void execute() {
        dt.resetGyro();
        dt.resetOdo();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
