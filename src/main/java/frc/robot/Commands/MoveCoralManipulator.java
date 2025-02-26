package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CoralSubsystem;

public class MoveCoralManipulator extends Command {
    private final CoralSubsystem co;
    private final boolean extend;

    public MoveCoralManipulator(CoralSubsystem co, boolean extend) {
        this.co = co;
        this.extend = extend;
        addRequirements(co);
    }

    @Override
    public void initialize() {
        System.out.println("MoveCoralManipulator initialized");
    }

    @Override
    public void execute() {
        if (extend) {
            co.extendManipulator();
            System.out.println("Manipulator is being extended");
        } else {
            co.retractManipulator();
            System.out.println("Manipulator is being retracted");
        }
    }

    @Override
    public void end(boolean interrupted) {
        co.stopMotor();
        System.out.println("Manipulator stopped, holding position.");
    }

    @Override
    public boolean isFinished() {
        return false; // Ensures it runs continuously while button is held
    }
}
