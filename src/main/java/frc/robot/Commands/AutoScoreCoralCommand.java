package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Subsystems.vision.Vision;
import frc.robot.Subsystems.coral.CoralSubsystem;
import frc.robot.Subsystems.drive.Drivetrain;
import frc.robot.Subsystems.elevator.ElevatorSubsystem;

class AutoScoreCoralCommand extends SequentialCommandGroup {
    public AutoScoreCoralCommand(Drivetrain dt, ElevatorSubsystem elevator, Vision vision, CoralSubsystem coral) {
        vision.setFiducialIDFilter(0, Constants.ID.reefAprilIDs);
        addCommands(
                new CenterOnTag(dt, vision),

                elevator.runOnce(() -> elevator.setLevelUsingFlag()),

                coral.runOnce(() -> coral.extendManipulator()),

                coral.runOnce(() -> coral.ejectCoral()));
        vision.setFiducialIDFilter(0, Constants.ID.allAprilIDs);

    }
}
