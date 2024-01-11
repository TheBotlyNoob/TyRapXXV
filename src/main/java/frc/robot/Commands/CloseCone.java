package frc.robot.Commands;

import frc.robot.Subsystems.Pneumatics;
import edu.wpi.first.wpilibj2.command.Command;

public class CloseCone extends Command {
    private Pneumatics m_pneumaticSystem;

    public CloseCone(Pneumatics pneumatics) {
        m_pneumaticSystem = pneumatics;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_pneumaticSystem.disableCone();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

}