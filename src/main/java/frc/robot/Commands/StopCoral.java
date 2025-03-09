package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CoralSubsystem;

public class StopCoral extends Command {
    CoralSubsystem co;

    public StopCoral(CoralSubsystem co) {
        this.co = co;
    }

    @Override
    public void initialize() {
        System.out.println("Stopping the coral.");
        co.stopMotorGrabber();
        co.stopMotorWrist();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
