package frc.robot.Subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Utils.SafeableSubsystem;
import frc.robot.Utils.TrapezoidController;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class ElevatorSubsystem extends SafeableSubsystem {
    @AutoLogOutput(key = "ElevatorSubsystem/Level")
    private ElevatorLevel m_level = ElevatorLevel.GROUND;
    @AutoLogOutput(key = "ElevatorSubsystem/LevelFlag")
    private ElevatorLevel m_levelFlag = ElevatorLevel.GROUND;
    @AutoLogOutput
    protected boolean isAnyLevelSet = false;

    private final ElevatorMotorIO m_motorIo;
    private final ElevatorMotorIOInputsAutoLogged m_motorInputs = new ElevatorMotorIOInputsAutoLogged();

    private final ElevatorFeedforward m_feedforward;
    private final PIDController m_pidController;
    protected final TrapezoidController m_trapController;

    // make sure limit switches are not unhooked or it freaks out
    protected final ElevatorLimitsIO m_limitsIo;
    protected final ElevatorLimitsIOInputsAutoLogged m_limitsInputs = new ElevatorLimitsIOInputsAutoLogged();

    protected final ElevatorConfigIO m_configIo;
    protected final ElevatorConfigIOInputsAutoLogged m_configInputs = new ElevatorConfigIOInputsAutoLogged();

    protected double outputVoltage = 0;

    @AutoLogOutput
    protected double desiredPositionRot = 0;

    protected double currentPositionRot = 0;
    protected double currentVelocityRotPerSec = 0;

    @AutoLogOutput
    protected double targetVelocityRotPerSec = 0;

    protected double m_lastSpeed = 0.0;
    protected double m_lastTime = 0.0;
    protected boolean m_testMode = false;
    protected boolean m_manualMode = false;
    protected double m_manualSpeed = 0.0;
    protected double m_lastDesiredPosition = 0.0;

    protected LoggedMechanismLigament2d mechanism = new LoggedMechanismLigament2d(getName(), 0.0, 90);

    public ElevatorSubsystem(ElevatorMotorIO motorIo,
            ElevatorLimitsIO limitsIo, ElevatorConfigIO configIo) {
        m_configIo = configIo;
        m_motorIo = motorIo;
        m_limitsIo = limitsIo;

        m_motorIo.resetLeaderEncoderPosition();

        m_feedforward = new ElevatorFeedforward(
                Constants.Elevator.FF.kS,
                Constants.Elevator.FF.kG,
                Constants.Elevator.FF.kV,
                Constants.Elevator.FF.kA);

        m_pidController = new PIDController(
                Constants.Elevator.PID.kP,
                Constants.Elevator.PID.kI,
                Constants.Elevator.PID.kD);

        m_trapController = new TrapezoidController(0.0, 0.05, .1, Constants.Elevator.kMaxVelocity,
                Constants.Elevator.kMaxAcceleration, 7.5,
                Constants.Elevator.kDecelProp);

        setLevel(ElevatorLevel.GROUND);
    }

    public void setLevel(ElevatorLevel level) {
        System.out.println("Calling set level to " + level);
        m_level = level;
        desiredPositionRot = this.m_level.toHeight();
    }

    public void setLevelFlag(ElevatorLevel level) {
        m_levelFlag = level;
        isAnyLevelSet = true;
        System.out.println("Setting elevator level flag to: " + m_levelFlag);
    }

    public void setLevelUsingFlag() {
        // m_level = m_levelFlag;
        // m_table_level.set(m_level.toString());
        this.setLevel(m_levelFlag);
        System.out.println("Moving elevator using the flag to: " + m_level);
    }

    public ElevatorLevel getLevelFlag() {
        return m_levelFlag;
    }

    public boolean isValidAlgaeLevel() {
        return m_levelFlag == ElevatorLevel.LEVEL1 || m_levelFlag == ElevatorLevel.LEVEL3;
    }

    public boolean isAnyLevelSet() {
        return isAnyLevelSet;
    }

    public void levelUp() {
        int currentLevel = ElevatorLevel.toInt(m_level);
        if (currentLevel < ElevatorLevel.toInt(ElevatorLevel.LEVEL4)) {
            setLevel(ElevatorLevel.fromInt(currentLevel + 1));
        }
    }

    public void levelDown() {
        int currentLevel = ElevatorLevel.toInt(m_level);
        if (currentLevel > ElevatorLevel.toInt(ElevatorLevel.GROUND)) {
            setLevel(ElevatorLevel.fromInt(currentLevel - 1));
        }
    }

    public double getDesiredPositionRot() {
        return this.desiredPositionRot;
    }

    public ElevatorFeedforward getFeedforward() {
        return m_feedforward;
    }

    public PIDController getPID() {
        return m_pidController;
    }

    public ElevatorLevel getLevel() {
        return m_level;
    }

    public double getCurrentPositionRot() {
        return currentPositionRot;
    }

    public void holdCurrentPosition() {
        desiredPositionRot = currentPositionRot;
    }

    public double getCurrentVelocityRotPerSec() {
        return currentVelocityRotPerSec;
    }

    public void resetEncoder() {
        if (m_limitsInputs.touchingBottom) {
            m_motorIo.resetLeaderEncoderPosition();
            setLevel(ElevatorLevel.GROUND);
        } else {
            System.err.println("Tried to reset elevator encoder when not at ground level");
        }
    }

    public void updateConstants() {
        m_feedforward.setKs(m_configInputs.feedForward.Ks);
        m_feedforward.setKg(m_configInputs.feedForward.Kg);
        m_feedforward.setKv(m_configInputs.feedForward.Kv);
        m_feedforward.setKa(m_configInputs.feedForward.Ka);
        m_pidController.setP(m_configInputs.PID.Kp);
        m_pidController.setI(m_configInputs.PID.Ki);
        m_pidController.setD(m_configInputs.PID.Kd);
        m_trapController.setMaxVel(m_configInputs.maxVelocity.in(Units.RotationsPerSecond));
        m_trapController.setMaxAccel(m_configInputs.maxAccel.in(Units.RotationsPerSecondPerSecond));
        m_trapController.setDecelKp(m_configInputs.decelerationProportion);

        ElevatorLevel.LEVEL1.setHeight(m_configInputs.heights.level1);
        ElevatorLevel.LEVEL2.setHeight(m_configInputs.heights.level2);
        ElevatorLevel.LEVEL3.setHeight(m_configInputs.heights.level3);
        ElevatorLevel.LEVEL4.setHeight(m_configInputs.heights.level4);
    }

    public void setVoltageTest(double voltage) {
        outputVoltage = voltage;
        if (voltage > 0.0) {
            targetVelocityRotPerSec = 0.1;
        } else if (voltage < 0.0) {
            targetVelocityRotPerSec = -0.1;
        } else {
            targetVelocityRotPerSec = 0.0;
        }
        handleLimits();
        m_motorIo.setLeaderVoltage(outputVoltage);
    }

    public void manualUp() {
        m_manualMode = true;
        m_manualSpeed = Constants.Elevator.kManualSpeed;
    }

    public void manualDown() {
        m_manualMode = true;
        m_manualSpeed = -Constants.Elevator.kManualSpeed;
    }

    public void stopManualMode() {
        m_manualMode = false;
    }

    public void setTestMode(boolean testMode) {
        this.m_testMode = testMode;
    }

    public void handleLimits() {
        if (m_limitsInputs.touchingBottom) {
            if (outputVoltage < 0 || targetVelocityRotPerSec <= 0.0) {
                outputVoltage = 0;
            }
        }
        if (m_limitsInputs.touchingTop) {
            if (outputVoltage > 0) {
                outputVoltage = 0;
            }
        }
        if (currentPositionRot > Constants.Elevator.kElevatorMaxPos && outputVoltage > 0.0) {
            System.out.println("Cut off output due to max height");
            outputVoltage = 0.4;
        }
    }

    public void makeSafe() {
        setLevel(ElevatorLevel.GROUND);
    }

    public LoggedMechanismLigament2d getMechanism() {
        return mechanism;
    }

    @Override
    public void periodic() {
        m_configIo.updateInputs(m_configInputs);
        Logger.processInputs("ElevatorSubsystem/Config", m_configInputs);

        m_motorIo.updateInputs(m_motorInputs);
        Logger.processInputs("ElevatorSubsystem/Motors", m_motorInputs);

        m_limitsIo.updateInputs(m_limitsInputs);
        Logger.processInputs("ElevatorSubsystem/Limits", m_limitsInputs);

        // TODO: encoder units -> meters
        mechanism.setLength(m_motorInputs.leaderRelativeEncoderPosition.in(Units.Rotations));

        if (!this.m_testMode) {
            currentPositionRot = m_motorInputs.leaderRelativeEncoderPosition.in(Units.Rotations);
            double currentDelta = desiredPositionRot - currentPositionRot;
            currentVelocityRotPerSec = m_motorInputs.leaderRelativeEncoderVelocity.in(Units.RotationsPerSecond);
            targetVelocityRotPerSec = m_trapController.calculate(currentDelta, currentVelocityRotPerSec);

            if (desiredPositionRot != this.m_lastDesiredPosition) {
                if (Math.abs(currentVelocityRotPerSec) < 0.001) {
                    System.out.println("Reinit trap controller");
                    m_trapController.reinit(currentVelocityRotPerSec);
                }
            }

            if (m_manualMode) {
                if (currentPositionRot >= ElevatorLevel.LEVEL4.toHeight()) {
                    m_level = ElevatorLevel.LEVEL4;
                } else if (currentPositionRot >= ElevatorLevel.LEVEL3.toHeight()) {
                    m_level = ElevatorLevel.LEVEL3;
                } else if (currentPositionRot >= ElevatorLevel.LEVEL2.toHeight()) {
                    m_level = ElevatorLevel.LEVEL2;
                } else if (currentPositionRot >= ElevatorLevel.LEVEL1.toHeight()) {
                    m_level = ElevatorLevel.LEVEL1;
                } else {
                    m_level = ElevatorLevel.GROUND;
                }
                desiredPositionRot = currentPositionRot;
                targetVelocityRotPerSec = m_manualSpeed;
            }
            // Enforce a velocity limit for safety until tuning complete
            targetVelocityRotPerSec = MathUtil.clamp(targetVelocityRotPerSec, -2.5, 1.8);

            double targetAcceleration = (targetVelocityRotPerSec - this.m_lastSpeed);

            double pidVal = m_pidController.calculate(currentPositionRot, desiredPositionRot);
            // TODO: the suggested alternative, calculateWithVelocities, ruins the FF. Needs further investigation.
            double FFVal = m_feedforward.calculate(targetVelocityRotPerSec, targetAcceleration);

            outputVoltage = pidVal + FFVal;
            this.m_lastSpeed = currentVelocityRotPerSec;
            this.m_lastTime = Timer.getFPGATimestamp();

            handleLimits();

            this.m_lastDesiredPosition = desiredPositionRot;

            Logger.recordOutput("ElevatorSubsystem/OutputVoltage/PreClamp", outputVoltage);

            if (currentPositionRot < 3) {
                outputVoltage = MathUtil.clamp(outputVoltage, -1.0, 6.0);
            } else if (currentPositionRot > Constants.Elevator.kElevatorMaxPos - 1.5) {
                outputVoltage = MathUtil.clamp(outputVoltage, -2.0, 1.0);
            } else {
                outputVoltage = MathUtil.clamp(outputVoltage, -4.0, 6.0);
            }

            Logger.recordOutput("ElevatorSubsystem/OutputVoltage/PostClamp", outputVoltage);

            m_motorIo.setLeaderVoltage(outputVoltage);
        }
    }

    public LoggedMechanismLigament2d mechanism() {
        return new LoggedMechanismLigament2d(getName(), currentPositionRot, 0.0);
    }
}
