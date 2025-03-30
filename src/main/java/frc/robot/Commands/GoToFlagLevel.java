package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
        // TODO: actually simulate elevator
        if (Constants.RobotMode.currentMode == Constants.RobotMode.Mode.SIM)
            return true;

        if (Math.abs(el.getCurrentPositionRot() - el.getDesiredPositionRot()) < 0.3) {
            if (Math.abs(el.getCurrentVelocityRotPerSec()) < 0.2) {
                System.out.println("Elevator has reached desired level");
                return true;
            }
        }
        return false;
    }
}
