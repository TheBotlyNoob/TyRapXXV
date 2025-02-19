package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ClimberSubsystem;

public class Climber extends Command {
    ClimberSubsystem cl;
    public Climber(ClimberSubsystem cl) {
        this.cl = cl;
        addRequirements(cl);
    }

    @Override
    public void execute() {
        cl.climb();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

