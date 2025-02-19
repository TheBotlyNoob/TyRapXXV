package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ClimberSubsystem;

public class ClimbGrabCage extends Command {
    ClimberSubsystem cl;
    public ClimbGrabCage(ClimberSubsystem cl) {
        this.cl = cl;
        addRequirements(cl);
    }

    @Override
    public void execute() {
        cl.moveArmsIn();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

