package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ClimberSubsystem;
public class MoveStinger extends Command {
    ClimberSubsystem cl;
    boolean extend;
    public MoveStinger(ClimberSubsystem cl, boolean extend) {
        this.cl = cl;
        this.extend = extend;
        addRequirements(cl);
    }

    @Override
    public void execute() {
        if (extend){
            cl.extendStinger();
        }
        else {
            cl.retractStinger();
        }
    }

    @Override
    public void end(boolean interupted) {
        cl.stopMotor();
    }
}
