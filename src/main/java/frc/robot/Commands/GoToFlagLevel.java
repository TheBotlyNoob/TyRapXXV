package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ElevatorSubsystem;

public class GoToFlagLevel extends Command {
    ElevatorSubsystem el;
    private boolean cancelFlag;

    public GoToFlagLevel(ElevatorSubsystem el) {
        this.el = el;
    }

    @Override
    public void initialize() {
        cancelFlag = false;
        el.setLevelUsingFlag();
    }

    @Override
    public boolean isFinished() {
        if(cancelFlag == true){
            return true;
        }
        if (Math.abs(el.getCurrentPosition() - el.getDesiredPosition()) < 0.2){
            if (Math.abs(el.getCurrentVelocity()) < 0.2){
                System.out.println("elevator has reached desired level");
                return true;
            }
        }
        return false; 
    }
    public void stopComand(){
        cancelFlag = true;
    }
}
