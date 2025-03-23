package frc.robot.Subsystems.coral;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.MoveCoralManipulator;
import frc.robot.Subsystems.elevator.ElevatorSubsystem;
import frc.robot.Subsystems.elevator.ElevatorSubsystem.ElevatorLevel;
import frc.robot.Utils.SafeableSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;

/*
 * 
 * different speeds up vs down 
 * IR sensor 
 * holde break logic
 */
public class CoralSubsystem extends SafeableSubsystem {
    private final ElevatorSubsystem el;

    private boolean pointedOut = false;
    private Rotation2d holdPosition = new Rotation2d();

    private final CoralDetectionIO m_detectionIo;
    private final CoralDetectionIOInputsAutoLogged m_detectionInputs = new CoralDetectionIOInputsAutoLogged();

    private final CoralWristIO m_wristIo;
    private final CoralWristIOInputsAutoLogged m_wristInputs = new CoralWristIOInputsAutoLogged();

    private final CoralConfigIO m_configIo;
    private final CoralConfigIOInputsAutoLogged m_configInputs = new CoralConfigIOInputsAutoLogged();

    // AKA ejector
    private final CoralGrabberIO m_grabberIo;
    private final CoralGrabberIOInputsAutoLogged m_grabberInputs = new CoralGrabberIOInputsAutoLogged();

    private Rotation2d start;
    private boolean ejectActive = false;
    private Timer timer;

    public enum CoralState {
        WAITING, INTAKING, HOLDING, EJECTING;
    };

    @AutoLogOutput
    protected CoralState state = CoralState.WAITING;
    protected boolean enabled = false;

    protected final LoggedMechanism2d coralSystemMechanism = new LoggedMechanism2d(20,
            20);
    protected final LoggedMechanismLigament2d ejectorMechanism = new LoggedMechanismLigament2d(getName(), 10, 90);

    public CoralSubsystem(ElevatorSubsystem el, CoralGrabberIO grabberIo, CoralWristIO wristIo,
            CoralDetectionIO detectionIo,
            CoralConfigIO configIo) {
        this.el = el;
        m_detectionIo = detectionIo;
        m_wristIo = wristIo;
        m_configIo = configIo;
        m_grabberIo = grabberIo;

        coralSystemMechanism.getRoot(getName(), 4, 4).append(el.getMechanism()).append(ejectorMechanism);

        timer = new Timer();
    }

    public void ejectCoral() {
        ejectActive = true;
        m_grabberIo.setSpeed(0.5);
    }

    // public void setVoltageTest(double voltage) {
    // System.out.println("Setting Coral voltage " + voltage);
    // m_wristMotor.setVoltage(voltage);
    // }

    public void reverseMotor() {
        // double speed = kWristMotorSpeedReverse.get();
        // m_wristMotor.set(-speed);
        /*
         * if (m_wristEncoder.getPosition() <= Constants.Coral.kMinEncoderPos) {
         * stopMotorWrist();
         * } else {
         * m_wristMotor.setVoltage(-voltage);
         * }
         */

        m_wristIo.setVoltage(m_configInputs.wristMotorVoltageReverse.unaryMinus());

    }

    public void forwardMotor() {
        Voltage volts = m_configInputs.wristMotorVoltageForward;
        // double speed = kWristMotorSpeedForward.get();
        /*
         * if (m_wristEncoder.getPosition() >= Constants.Coral.kMaxEncoderPos) {
         * stopMotorWrist();
         * } else {
         * m_wristMotor.setVoltage(voltage);
         * }
         */

        m_wristIo.setVoltage(volts);

        // m_wristMotor.set(speed);
    }

    public void stopMotorWrist() {
        holdPosition = m_wristInputs.absoluteEncoderPosition;
        m_wristIo.setVoltage(Units.Volts.of(0.0));
        // m_wristMotor.set(0.0);
    }

    public void stopMotorGrabber() {
        m_grabberIo.setSpeed(0.0);
    }

    public void extendManipulator() {
        forwardMotor();
    }

    public void retractManipulator() {
        reverseMotor();
    }

    public Command wristExtendCommand() {
        return new MoveCoralManipulator(this, true).withTimeout(0.8);
    }

    public Command wristRetractCommand() {
        return new MoveCoralManipulator(this, false).withTimeout(0.8);
    }

    public void reinit() {
        if (m_detectionInputs.hasCoral) {
            state = CoralState.HOLDING;
            el.setLevel(ElevatorLevel.LEVEL1);
            m_grabberIo.setSpeed(0.0);
        } else {
            state = CoralState.WAITING;
        }
        enabled = true;
    }

    public CoralState getState() {
        return state;
    }

    @Override
    public void periodic() {
        m_detectionIo.updateInputs(m_detectionInputs);
        Logger.processInputs("CoralSubsystem/Detection", m_detectionInputs);

        m_grabberIo.updateInputs(m_grabberInputs);
        Logger.processInputs("CoralSubsystem/Grabber", m_grabberInputs);

        m_wristIo.updateInputs(m_wristInputs);
        Logger.processInputs("CoralSubsystem/Wrist", m_wristInputs);

        m_configIo.updateInputs(m_configInputs);
        Logger.processInputs("CoralSubsystem/Config", m_configInputs);

        Logger.recordOutput("CoralSubsystem/MechanismView", coralSystemMechanism);

        if (!enabled) {
            return;
        }

        if (state == CoralState.WAITING) {
            if (m_detectionInputs.hasCoral) {
                state = CoralState.INTAKING;
                start = m_grabberInputs.relativeEncoderPosition;
            } else {
                m_grabberIo.setSpeed(0.1);
            }
        } else if (state == CoralState.INTAKING) {
            m_grabberIo.setSpeed(0.1);
            if (m_grabberInputs.relativeEncoderPosition.getRotations() >= (start.getRotations() + 4.5)) {
                state = CoralState.HOLDING;
                el.setLevel(ElevatorLevel.LEVEL1);
            }
        } else if (state == CoralState.HOLDING) {
            m_grabberIo.setSpeed(0.0);
            if (ejectActive) {
                state = CoralState.EJECTING;
                timer.start();
            } else if (!m_detectionInputs.hasCoral) {
                // We thought we were holding a piece but no longer see one
                state = CoralState.WAITING;
            }
        } else if (state == CoralState.EJECTING) {
            m_grabberIo.setSpeed(0.5);
            if (timer.get() > 2 && !m_detectionInputs.hasCoral) {
                state = CoralState.WAITING;
                ejectActive = false;
            }
        }
    }

    @Override
    public void makeSafe() {
        Command retract = this.wristRetractCommand();
        retract.schedule();
        stopMotorGrabber();
    }
}
