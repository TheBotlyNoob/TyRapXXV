package frc.robot.Subsystems;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
                    return Constants.ElevatorHeights.kGround;
                case LEVEL1:
                    return Constants.ElevatorHeights.kLevel1;
                case LEVEL2:
                    return Constants.ElevatorHeights.kLevel2;
                case LEVEL3:
                    return Constants.ElevatorHeights.kLevel3;
                case LEVEL4:
                    return Constants.ElevatorHeights.kLevel4;
                default:
                    return Constants.ElevatorHeights.kGround;
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
                    return String.format("LEVEL1 (%.2f rotations)", Constants.ElevatorHeights.kLevel1);
                case LEVEL2:
                    return String.format("LEVEL2 (%.2f rotations)", Constants.ElevatorHeights.kLevel2);
                case LEVEL3:
                    return String.format("LEVEL3 (%.2f rotations)", Constants.ElevatorHeights.kLevel3);
                case LEVEL4:
                    return String.format("LEVEL4 (%.2f rotations)", Constants.ElevatorHeights.kLevel4);
                default:
                    return String.format("GROUND (%.2f rotations)", Constants.ElevatorHeights.kGround);
            }
        }
    }

    private ElevatorLevel m_level = ElevatorLevel.GROUND;

    private final NetworkTable m_table;
    private final StringPublisher m_table_level;

    // TODO: fill these publishers
    // private final DoublePublisher m_table_motor1_current;
    // private final DoublePublisher m_table_motor2_current;
    // private final DoublePublisher m_table_motor1_voltage;
    // private final DoublePublisher m_table_motor2_voltage;
    // private final DoublePublisher m_table_motor1_position;
    // private final DoublePublisher m_table_motor2_position;

    // private final SparkFlex m_motor1;
    // private final SparkFlex m_motor2;

    public ElevatorSubsystem(NetworkTableInstance nt) {
        m_table = nt.getTable(getName());
        m_table_level = m_table.getStringTopic("level").publish();
    }

    public void setLevel(ElevatorLevel level) {
        m_level = level;
        m_table_level.set(level.toString());
        setMotors();
    }

    public ElevatorLevel getLevel() {
        return m_level;
    }

    private void setMotors() {
    }
}
