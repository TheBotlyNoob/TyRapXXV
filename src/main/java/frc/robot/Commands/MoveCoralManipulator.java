package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CoralSubsystem;
public class MoveCoralManipulator extends Command {
    CoralSubsystem co;
    boolean extend;
    public MoveCoralManipulator(CoralSubsystem co, boolean extend) {
        this.co = co;
        this.extend = extend;
        addRequirements(co);
    }

    @Override
    public void execute() {
        if (extend){
            co.extendManipulator();
            System.out.println("Manipulator is being extended");
        }
        else {
            co.retractManipulator();
            System.out.println("Manipulator is being retracted");

        }
    }

    @Override
    public void end(boolean interupted) {
        co.stopMotor();
    }
}
