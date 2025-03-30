package frc.robot.Subsystems.climber;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils.SafeableSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// this doesn't make sense as a `SafeableSubsystem`
public class ClimberSubsystem extends SubsystemBase {
    @AutoLogOutput
    protected boolean isClimbMode = false;

    protected final SafeableSubsystem[] m_toMakeSafe;

    protected final ClimberStingerIO m_stingerIo;
    protected final ClimberStingerIOInputsAutoLogged m_stingerInputs = new ClimberStingerIOInputsAutoLogged();

    protected final ClimberPneumaticsIO m_pneumaticsIo;
    protected final ClimberPneumaticsIOInputsAutoLogged m_pneumaticsInputs = new ClimberPneumaticsIOInputsAutoLogged();

    public ClimberSubsystem(ClimberStingerIO stingerIo, ClimberPneumaticsIO pneumaticsIo, SafeableSubsystem[] toMakeSafe) {
        m_stingerIo = stingerIo;
        m_pneumaticsIo = pneumaticsIo;

        m_pneumaticsIo.setArmState(ClimberPneumaticsIO.ArmState.RETRACTED);
        m_pneumaticsIo.setGrabberState(ClimberPneumaticsIO.GrabberState.OPEN);
        m_pneumaticsIo.setRampState(ClimberPneumaticsIO.RampState.UP);

        m_toMakeSafe = toMakeSafe;
    }

    public boolean isClimbMode() {
        return isClimbMode;
    }

    public boolean isCoralMode() {
        return !isClimbMode;
    }

    public void toggleClimbMode() {
        System.out.println("Toggling climb current=" + isClimbMode);
        if (isClimbMode) {
            setCoralMode();
        } else {
            setClimbMode();
        }
    }

    public void setClimbMode() {
        extendArms();
        rampUp();
        m_pneumaticsIo.setGrabberState(ClimberPneumaticsIO.GrabberState.OPEN);

        for (SafeableSubsystem safeableSubsystem : m_toMakeSafe) {
            System.out.println("Placing " + safeableSubsystem.getName() + " into a safe position");
            safeableSubsystem.makeSafe();
        }
        isClimbMode = true;
    }

    public void setCoralMode() {
        retractArms();
        rampDown();
        m_pneumaticsIo.setGrabberState(ClimberPneumaticsIO.GrabberState.OPEN);
        isClimbMode = false;
    }

    public void toggleGrabArms() {
        System.out.println("Toggling climb grabber current=" + m_pneumaticsInputs.grabberState.toString());
        m_pneumaticsIo.setGrabberState(
                m_pneumaticsInputs.grabberState == ClimberPneumaticsIO.GrabberState.OPEN
                        ? ClimberPneumaticsIO.GrabberState.CLOSED
                        : ClimberPneumaticsIO.GrabberState.OPEN
        );
    }

    public void extendArms() {
        m_pneumaticsIo.setArmState(ClimberPneumaticsIO.ArmState.EXTENDED);
    }

    public void retractArms() {
        m_pneumaticsIo.setArmState(ClimberPneumaticsIO.ArmState.RETRACTED);
    }

    public void rampUp() {
        m_pneumaticsIo.setRampState(ClimberPneumaticsIO.RampState.UP);
    }

    public void rampDown() {
        m_pneumaticsIo.setRampState(ClimberPneumaticsIO.RampState.DOWN);
    }

    public void stopStinger() {
        m_stingerIo.setVoltage(0.0);
    }

    public void extendStinger() {
        System.out.println("forwardMotor called");

        if (m_stingerInputs.dutyCycleEncoderPosition.in(Units.Rotations) >= Constants.Climber.kMaxEncoderPos) {
            stopStinger();
        } else {
            m_stingerIo.setVoltage(Constants.Climber.kClimbMotorVoltage);
        }
    }

    public void retractStinger() {
        System.out.println("reverseMotor called");

        if (m_stingerInputs.dutyCycleEncoderPosition.in(Units.Rotations) <= Constants.Climber.kMinEncoderPos) {
            stopStinger();
        } else {
            m_stingerIo.setVoltage(-Constants.Climber.kClimbMotorVoltage);
        }
    }

    @Override
    public void periodic() {
        m_stingerIo.updateInputs(m_stingerInputs);
        Logger.processInputs("ClimberSubsystem/Stinger", m_stingerInputs);

        m_pneumaticsIo.updateInputs(m_pneumaticsInputs);
        Logger.processInputs("ClimberSubsystem/Pneumatics", m_pneumaticsInputs);
    }
}
