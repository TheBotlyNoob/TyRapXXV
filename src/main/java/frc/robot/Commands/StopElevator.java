package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ElevatorSubsystem;

public class StopElevator extends Command {
    ElevatorSubsystem el;

    public StopElevator(ElevatorSubsystem el) {
        this.el = el;
    }

    @Override
    public void initialize() {
        System.out.println("Stopping the elevator.");
        el.holdCurrentPosition();
    }

    @Override
    public boolean isFinished() {
        return true; 
    }
}


