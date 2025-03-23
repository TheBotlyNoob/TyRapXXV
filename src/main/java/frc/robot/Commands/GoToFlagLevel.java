package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.elevator.ElevatorSubsystem;

public class GoToFlagLevel extends Command {
    ElevatorSubsystem el;

    public GoToFlagLevel(ElevatorSubsystem el) {
        this.el = el;
    }

    @Override
    public void initialize() {
        el.setLevelUsingFlag();
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(el.getCurrentPositionRot() - el.getDesiredPositionRot()) < 0.3) {
            if (Math.abs(el.getCurrentVelocityRotPerSec()) < 0.2) {
                System.out.println("Elevator has reached desired level");
                return true;
            }
        }
        return false;
    }
}
