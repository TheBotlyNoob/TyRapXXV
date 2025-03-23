package frc.robot.Commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class RumbleController extends Command {
    private final XboxController m_controller;
    protected final Timer timer = new Timer();
    protected final double durationSec;

    public RumbleController(XboxController controller, double durationSec) {
        this.durationSec = durationSec;
        m_controller = controller;

        timer.start();

    }

    @Override
    public void initialize() {
        m_controller.setRumble(RumbleType.kLeftRumble, 0.3);
        m_controller.setRumble(RumbleType.kRightRumble, 0.3);
    }

    @Override
    public void end(boolean interrupted) {
        m_controller.setRumble(RumbleType.kLeftRumble, 0.0);
        m_controller.setRumble(RumbleType.kRightRumble, 0.0);
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= durationSec;
    }
}
