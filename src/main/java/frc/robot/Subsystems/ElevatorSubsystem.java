package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.MotorPublisher;

public class ElevatorSubsystem extends SubsystemBase {
    public enum ElevatorLevel {
        /**
         * The ground level of the elevator, where the human player can load coral.
         */
        GROUND,

        /**
         * AKA the trough level
         */
        LEVEL1,

        LEVEL2,
        LEVEL3,
        LEVEL4;

        /**
         * Converts an integer to an ElevatorLevel
         * 
         * @param i
         * @return the ElevatorLevel corresponding to the integer, or GROUND if the
         *         integer is not recognized
         */
        public static ElevatorLevel fromInt(int i) {
            switch (i) {
                case 0:
                    return GROUND;
                case 1:
                    return LEVEL1;
                case 2:
                    return LEVEL2;
                case 3:
                    return LEVEL3;
                case 4:
                    return LEVEL4;
                default:
                    return GROUND;
            }
        }

        /**
         * Converts an ElevatorLevel to an integer
         * 
         * @param level
         * @return the integer corresponding to the ElevatorLevel
         */
        public static int toInt(ElevatorLevel level) {
            switch (level) {
                case GROUND:
                    return 0;
                case LEVEL1:
                    return 1;
                case LEVEL2:
                    return 2;
                case LEVEL3:
                    return 3;
                case LEVEL4:
                    return 4;
                default:
                    return 0;
            }
        }

        /**
         * Converts an ElevatorLevel to its height in encoder-specific
         * 'rotations'/'ticks'
         * 
         * @return the height of the ElevatorLevel in encoder-specific
         *         'rotations'/'ticks'
         */
        public double toHeight() {
            switch (this) {
                case GROUND:
                    return Constants.Elevator.Heights.kGround;
                case LEVEL1:
                    return Constants.Elevator.Heights.kLevel1;
                case LEVEL2:
                    return Constants.Elevator.Heights.kLevel2;
                case LEVEL3:
                    return Constants.Elevator.Heights.kLevel3;
                case LEVEL4:
                    return Constants.Elevator.Heights.kLevel4;
                default:
                    return Constants.Elevator.Heights.kGround;
            }
        }

        /**
         * Converts an ElevatorLevel to a human-readable string
         * 
         * @return the human-readable string corresponding to the ElevatorLevel
         */
        public String toString() {
            switch (this) {
                case LEVEL1:
                    return String.format("LEVEL1 (%.2f rotations)", Constants.Elevator.Heights.kLevel1);
                case LEVEL2:
                    return String.format("LEVEL2 (%.2f rotations)", Constants.Elevator.Heights.kLevel2);
                case LEVEL3:
                    return String.format("LEVEL3 (%.2f rotations)", Constants.Elevator.Heights.kLevel3);
                case LEVEL4:
                    return String.format("LEVEL4 (%.2f rotations)", Constants.Elevator.Heights.kLevel4);
                default:
                    return String.format("GROUND (%.2f rotations)", Constants.Elevator.Heights.kGround);
            }
        }
    }

    private ElevatorLevel m_level = ElevatorLevel.GROUND;

    private final NetworkTable m_table;
    private final StringPublisher m_table_level;

    private final SparkFlex m_motorLeader;
    private final SparkFlex m_motorFollower;

    private final MotorPublisher m_motorPublisherLeader;
    private final MotorPublisher m_motorPublisherFollower;

    private final ElevatorFeedforward m_feedforward;
    private final ProfiledPIDController m_controller;

    public ElevatorSubsystem(NetworkTableInstance nt) {
        m_table = nt.getTable(getName());
        m_table_level = m_table.getStringTopic("level").publish();

        m_motorLeader = new SparkFlex(Constants.ID.kElevatorMotorLeaderCANID, MotorType.kBrushless);
        m_motorFollower = new SparkFlex(Constants.ID.kElevatorMotorLeaderCANID, MotorType.kBrushless);

        SparkBaseConfig motorConf = new SparkFlexConfig();
        motorConf.smartCurrentLimit(40);
        motorConf.idleMode(IdleMode.kBrake);
        motorConf.inverted(false);

        m_motorLeader.configure(motorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motorConf.follow(m_motorLeader, false);
        m_motorFollower.configure(motorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_motorPublisherLeader = new MotorPublisher(m_motorLeader, m_table, "leader");
        m_motorPublisherFollower = new MotorPublisher(m_motorFollower, m_table, "follower");

        m_feedforward = new ElevatorFeedforward(
                Constants.Elevator.FF.kS,
                Constants.Elevator.FF.kG,
                Constants.Elevator.FF.kV,
                Constants.Elevator.FF.kA);

        m_controller = new ProfiledPIDController(
                Constants.Elevator.PID.kP,
                Constants.Elevator.PID.kI,
                Constants.Elevator.PID.kD,
                new TrapezoidProfile.Constraints(Constants.Elevator.kMaxVelocity, Constants.Elevator.kMaxAcceleration));

        setLevel(ElevatorLevel.GROUND);
    }

    public void setLevel(ElevatorLevel level) {
        m_level = level;
        m_table_level.set(level.toString());

        m_feedforward.calculate(m_controller.getSetpoint().velocity);

        m_motorLeader.setVoltage(m_controller.calculate(m_motorLeader.getEncoder().getPosition(),
                m_motorLeader.getEncoder().getVelocity())
                + m_feedforward.calculate(m_controller.getSetpoint().velocity));
    }

    public ElevatorLevel getLevel() {
        return m_level;
    }

    @Override
    public void periodic() {
        m_motorPublisherLeader.update();
        m_motorPublisherLeader.update();
    }
}
