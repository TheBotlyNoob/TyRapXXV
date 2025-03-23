package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.elevator.ElevatorSubsystem;

public class ElevatorJoystick extends Command {
    ElevatorSubsystem el;
    private final XboxController m_controller;
    protected final double multiplier = -6;

    public ElevatorJoystick(XboxController controller, ElevatorSubsystem el) {
        this.el = el;
        m_controller = controller;
        System.out.println("ElevatorJoystick command initialized");
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
        if (interrupted) {
            System.out.println("ElevatorJoystick command interrupted");
        }
        this.el.setTestMode(false);
    }
}
