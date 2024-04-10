package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;

public class StopDrive extends Command {
    Drivetrain dt;

    public StopDrive(Drivetrain dt) {
        this.dt = dt;
    }

    @Override
    public void schedule() {
        System.out.println("Stopped Driving.");
        dt.stopDriving();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
