package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Drivetrain;

public class ResetOdoCommand extends InstantCommand {
    Drivetrain dt;

    public ResetOdoCommand(Drivetrain dt) {
        this.dt = dt;
    }
    
    @Override
    public void execute() {
        dt.resetOdo();
    }
}
