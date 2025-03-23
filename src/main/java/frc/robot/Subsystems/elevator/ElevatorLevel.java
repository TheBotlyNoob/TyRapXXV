package frc.robot.Subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;

import frc.robot.Constants;

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
                return ground;
            case LEVEL1:
                return level1;
            case LEVEL2:
                return level2;
            case LEVEL3:
                return level3;
            case LEVEL4:
                return level4;
            default:
                return ground;
        }
    }

    public void setHeight(double height) {
        switch (this) {
            case GROUND:
                ground = Constants.Elevator.Heights.kGround;
                break;
            case LEVEL1:
                level1 = height;
                break;
            case LEVEL2:
                level2 = height;
                break;
            case LEVEL3:
                level3 = height;
                break;
            case LEVEL4:
                level4 = height;
                break;
            default:
                ground = Constants.Elevator.Heights.kGround;
                break;
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
                return String.format("LEVEL1 (%.2f rotations)", level1);
            case LEVEL2:
                return String.format("LEVEL2 (%.2f rotations)", level2);
            case LEVEL3:
                return String.format("LEVEL3 (%.2f rotations)", level3);
            case LEVEL4:
                return String.format("LEVEL4 (%.2f rotations)", level4);
            default:
                return String.format("GROUND (%.2f rotations)", ground);
        }
    }

    @AutoLogOutput
    double ground = Constants.Elevator.Heights.kGround;
    @AutoLogOutput
    double level1 = Constants.Elevator.Heights.kLevel1;
    @AutoLogOutput
    double level2 = Constants.Elevator.Heights.kLevel2;
    @AutoLogOutput
    double level3 = Constants.Elevator.Heights.kLevel3;
    @AutoLogOutput
    double level4 = Constants.Elevator.Heights.kLevel4;
}
