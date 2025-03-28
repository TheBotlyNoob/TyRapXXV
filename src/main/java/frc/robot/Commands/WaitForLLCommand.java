package frc.robot.Commands;

import frc.robot.Subsystems.vision.Vision;

public class WaitForLLCommand {
    // Command to wait until limelight detects a valid
    // April tag. Can decorate with .withTimeout(timeout) to
    // prevent it from running forever if a tag is never detected
    protected final Vision vision;

    public WaitForLLCommand(Vision vision) {
        this.vision = vision;
    }

    public boolean isFinished() {
        return vision.isTargetValid(0);
    }

}
