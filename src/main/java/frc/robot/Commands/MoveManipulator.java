package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CoralSubsystem;
public class MoveManipulator extends Command {
    CoralSubsystem co;
    boolean extend;
    public MoveManipulator(CoralSubsystem co, boolean extend) {
        this.co = co;
        this.extend = extend;
        addRequirements(co);
    }

    @Override
    public void execute() {
        if (extend){
            co.extendManipulator();
        }
        else {
            co.retractManipulator();
        }
    }

    @Override
    public void end(boolean interupted) {
        co.stopMotor();
    }
}
