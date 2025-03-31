package frc.robot.Commands;

import frc.robot.Subsystems.elevator.ElevatorLevel;
import frc.robot.Subsystems.elevator.ElevatorSubsystem;

public class GoToFlagLevel extends GoToLevel {
    public GoToFlagLevel(ElevatorSubsystem el) {
        super(el, ElevatorLevel.GROUND); // this level doesn't matter, it will be overwritten in initialize()
    }

    @Override
    public void initialize() {
        super.level = el.getLevelFlag();
        super.initialize();
    }
}
