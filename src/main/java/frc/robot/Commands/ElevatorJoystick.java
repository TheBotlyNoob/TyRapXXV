package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Controller;
import frc.robot.Subsystems.ElevatorSubsystem;

public class ElevatorJoystick extends Command {
    ElevatorSubsystem el;
    private final XboxController m_controller = new XboxController(Controller.kManipControllerID);
    protected final double multiplier = -6;

    public ElevatorJoystick(ElevatorSubsystem el) {
        this.el = el;
    }

    @Override
    public void initialize() {
        this.el.setTestMode(true);
    }

    @Override
    public void execute() {
        double elSpeed = m_controller.getLeftY() * multiplier;
        el.setVoltageTest(elSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        this.el.setTestMode(false);
    }
}


