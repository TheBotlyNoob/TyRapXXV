package frc.robot.Commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;

public class RumbleDrive extends Command {
    private final XboxController m_controller = new XboxController(Controller.kDriveControllerID);
    protected double durationSec;
    protected double endTimeSec;
    public RumbleDrive(double durationSec) {
        this.durationSec = durationSec;
    }

    @Override
    public void initialize() {
        this.endTimeSec = Timer.getFPGATimestamp() + durationSec;
        m_controller.setRumble(RumbleType.kLeftRumble, 0.3);
        m_controller.setRumble(RumbleType.kRightRumble, 0.3);
    }

    @Override
    public boolean isFinished() {
        if (Timer.getFPGATimestamp() >= endTimeSec){
            m_controller.setRumble(RumbleType.kLeftRumble, 0.0);
            m_controller.setRumble(RumbleType.kRightRumble, 0.0);
            return true;
        };
        return false;
    }
}
