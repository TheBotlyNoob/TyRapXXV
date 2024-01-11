package frc.robot.Commands;

import edu.wpi.first.wpilibj.DSControlWord;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;

public class StopDrive extends Command {
    Drivetrain dt;
    private final DSControlWord m_word = new DSControlWord();

    public StopDrive(Drivetrain dt) {
        this.dt = dt;
    }

    @Override
    public void schedule() {
        System.out.println("Stopped Driving.");
    }
    
    @Override
    public void execute() {
        m_word.refresh();
        dt.stopDriving();
    }

    @Override
    public boolean isFinished() {
        return m_word.isTeleop();
    }
}
