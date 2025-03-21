package frc.robot.Subsystems.coral;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Commands.EjectCoral;
import frc.robot.Commands.MoveCoralManipulator;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem.ElevatorLevel;
import frc.robot.Subsystems.coral.IrSensorIO;
import frc.robot.Utils.MotorPublisher;
import frc.robot.Utils.SafeableSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleEntry;

/*
 * 
 * different speeds up vs down 
 * IR sensor 
 * holde break logic
 */
public class CoralSubsystem extends SafeableSubsystem {
    private final SparkMax m_coralGrabberMotor;
    private final ElevatorSubsystem el;

    private boolean pointedOut = false;
    private Rotation2d holdPosition = new Rotation2d();

    private final IrSensorIO m_irIo;
    private final IrSensorIOInputsAutoLogged m_irInputs = new IrSensorIOInputsAutoLogged();

    private final WristIO m_wristIo;
    private final WristIOInputsAutoLogged m_wristInputs = new WristIOInputsAutoLogged();

    protected DoubleEntry kWristMotorVoltageForward;
    protected DoubleEntry kWristMotorVoltageReverse;
    protected DoubleEntry kWristMotorSpeedForward;
    protected DoubleEntry kWristMotorSpeedReverse;

    private double counter;
    private double start;
    private boolean ejectActive = false;
    private Timer timer;

    private Rotation2d lastWristEncoderVal;
    private boolean wristStopped = false;
    private int wristCounter = 0;

    public enum CoralState {
        WAITING, INTAKING, HOLDING, EJECTING;
    };

    protected CoralState state = CoralState.WAITING;
    protected boolean enabled = false;

    public CoralSubsystem(ElevatorSubsystem el, WristIO wristIo, IrSensorIO irIo) {
        this.el = el;
        m_irIo = irIo;
        m_wristIo = wristIo;

        m_coralGrabberMotor = new SparkMax(Constants.MechID.kCoralWheelCanId, MotorType.kBrushless);
        // m_wristMotor = new SparkMax(Constants.MechID.kCoralWristCanId,
        // MotorType.kBrushed);

        kWristMotorVoltageForward.set(Constants.Coral.kWristMotorVoltage);
        kWristMotorVoltageReverse.set(Constants.Coral.kWristMotorVoltageReverse);

        timer = new Timer();
    }

    public void ejectCoral() {
        ejectActive = true;
        m_coralGrabberMotor.set(0.5);
    }

    // public void setVoltageTest(double voltage) {
    // System.out.println("Setting Coral voltage " + voltage);
    // m_wristMotor.setVoltage(voltage);
    // }

    public void reverseMotor() {
        double voltage = kWristMotorVoltageReverse.get();
        // double speed = kWristMotorSpeedReverse.get();
        // m_wristMotor.set(-speed);
        /*
         * if (m_wristEncoder.getPosition() <= Constants.Coral.kMinEncoderPos) {
         * stopMotorWrist();
         * } else {
         * m_wristMotor.setVoltage(-voltage);
         * }
         */

        m_wristIo.setVoltage(Units.Volts.of(-voltage));

    }

    public void forwardMotor() {
        double voltage = kWristMotorVoltageForward.get();
        // double speed = kWristMotorSpeedForward.get();
        /*
         * if (m_wristEncoder.getPosition() >= Constants.Coral.kMaxEncoderPos) {
         * stopMotorWrist();
         * } else {
         * m_wristMotor.setVoltage(voltage);
         * }
         */

        m_wristIo.setVoltage(Units.Volts.of(voltage));

        // m_wristMotor.set(speed);
    }

    public void stopMotorWrist() {
        holdPosition = m_wristInputs.absoluteEncoderPosition;
        m_wristIo.setVoltage(Units.Volts.of(0.0));
        // m_wristMotor.set(0.0);
    }

    public void stopMotorGrabber() {
        m_coralGrabberMotor.set(0.0);
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
        if (m_irInputs.isBlocked) {
            state = CoralState.HOLDING;
            el.setLevel(ElevatorLevel.LEVEL1);
            m_coralGrabberMotor.set(0.0);
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
        m_irIo.updateInputs(m_irInputs);
        Logger.processInputs("CoralSubsystem/IrSensor", m_irInputs);

        m_wristIo.updateInputs(m_wristInputs);
        Logger.processInputs("CoralSubsystem/Wrist", m_wristInputs);

        if (!enabled) {
            return;
        }

        if (lastWristEncoderVal == m_wristInputs.absoluteEncoderPosition) {
            wristCounter++;
            if (counter >= Constants.Coral.wristCounterLimit) {
                wristStopped = true;
            }
        } else {
            wristStopped = false;
            wristCounter = 0;
        }

        if (state == CoralState.WAITING) {
            if (m_irInputs.isBlocked) {
                state = CoralState.INTAKING;
                counter = m_coralGrabberMotor.getEncoder().getPosition();
                start = counter;
            } else {
                m_coralGrabberMotor.set(0.1);
            }
        } else if (state == CoralState.INTAKING) {
            m_coralGrabberMotor.set(0.1);
            counter = m_coralGrabberMotor.getEncoder().getPosition();
            if (counter >= (start + 4.5)) {
                state = CoralState.HOLDING;
                el.setLevel(ElevatorLevel.LEVEL1);
            }
        } else if (state == CoralState.HOLDING) {
            m_coralGrabberMotor.set(0.0);
            if (ejectActive) {
                state = CoralState.EJECTING;
                timer.start();
            } else if (!m_irInputs.isBlocked) {
                // We thought we were holding a piece but no longer see one
                state = CoralState.WAITING;
            }
        } else if (state == CoralState.EJECTING) {
            m_coralGrabberMotor.set(0.5);
            if (timer.get() > 2 && !m_irInputs.isBlocked) {
                state = CoralState.WAITING;
                ejectActive = false;
            }
        }

        lastWristEncoderVal = m_wristInputs.absoluteEncoderPosition;
    }

    @Override
    public void makeSafe() {
        Command retract = this.wristRetractCommand();
        retract.schedule();
        stopMotorGrabber();
    }
}
