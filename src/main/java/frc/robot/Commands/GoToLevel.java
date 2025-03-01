package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem.ElevatorLevel;

public class GoToLevel extends Command {
    ElevatorSubsystem el;
    ElevatorLevel level;
    public GoToLevel(ElevatorSubsystem el, ElevatorLevel level) {
        this.el = el;
        this.level = level;
        
    }

    @Override
    public void initialize() {
        el.setLevel(level);
        System.out.println("setting desired elevator level to: " + level);
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(el.getCurrentPosition() - level.toHeight()) < 0.2){ // unknown tolerance to set
            if (Math.abs(el.getCurrentVelocity()) < 0.2){
                System.out.println("elevator has reached desired level");
                return true;
            }
        }
        return false; 
    }
}


