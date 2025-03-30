package frc.robot.Subsystems.climber;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class ClimberPneumaticsIOReal implements ClimberPneumaticsIO {
    private final DoubleSolenoid armSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Climber.kLowerSolenoidCANID1, Constants.Climber.kLowerSolenoidCANID2);
    private final DoubleSolenoid.Value kExtendArm = DoubleSolenoid.Value.kForward;
    private final DoubleSolenoid.Value kRetractArm = DoubleSolenoid.Value.kReverse;

    private final DoubleSolenoid grabberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Climber.kClampSolenoidCANID1, Constants.Climber.kClampSolenoidCANID2);
    private final DoubleSolenoid.Value kOpenGrabber = DoubleSolenoid.Value.kReverse;
    private final DoubleSolenoid.Value kCloseGrabber = DoubleSolenoid.Value.kForward;

    private final DoubleSolenoid rampSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Climber.kRampSolenoidCANID1, Constants.Climber.kRampSolenoidCANID2);
    private final DoubleSolenoid.Value kRampUp = DoubleSolenoid.Value.kForward;
    private final DoubleSolenoid.Value kRampDown = DoubleSolenoid.Value.kReverse;

    @Override
    public void updateInputs(ClimberPneumaticsIOInputs inputs) {
        inputs.armState = armSolenoid.get() == DoubleSolenoid.Value.kForward ? ArmState.EXTENDED : ArmState.RETRACTED;
        inputs.grabberState = grabberSolenoid.get() == DoubleSolenoid.Value.kForward ? GrabberState.CLOSED : GrabberState.OPEN;
        inputs.rampState = rampSolenoid.get() == DoubleSolenoid.Value.kForward ? RampState.UP : RampState.DOWN;
    }

    @Override
    public void setArmState(ArmState state) {
        switch (state) {
            case EXTENDED -> armSolenoid.set(kExtendArm);
            case RETRACTED -> armSolenoid.set(kRetractArm);
            default -> armSolenoid.set(DoubleSolenoid.Value.kOff);
        }
    }

    @Override
    public void setGrabberState(GrabberState state) {
        switch (state) {
            case OPEN -> grabberSolenoid.set(kOpenGrabber);
            case CLOSED -> grabberSolenoid.set(kCloseGrabber);
            default -> grabberSolenoid.set(DoubleSolenoid.Value.kOff);
        }
    }

    @Override
    public void setRampState(RampState state) {
        switch (state) {
            case UP -> rampSolenoid.set(kRampUp);
            case DOWN -> rampSolenoid.set(kRampDown);
            default -> rampSolenoid.set(DoubleSolenoid.Value.kOff);
        }
    }
}
