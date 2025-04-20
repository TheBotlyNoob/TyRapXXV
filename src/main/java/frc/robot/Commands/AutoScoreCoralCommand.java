package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ID;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.CoralSubsystem;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.Limelight;

class AutoScoreCoralCommand extends SequentialCommandGroup {
  public AutoScoreCoralCommand(
      Drivetrain dt, ElevatorSubsystem elevator, Limelight ll, CoralSubsystem coral) {
    LimelightHelpers.SetFiducialIDFiltersOverride(
        ID.kFrontLimelightName, Constants.ID.reefAprilIDs);
    addCommands(
        new CenterOnTag(dt, ll),
        elevator.runOnce(() -> elevator.setLevelUsingFlag()),
        coral.runOnce(() -> coral.extendManipulator()),
        coral.runOnce(() -> coral.ejectCoral()));
    LimelightHelpers.SetFiducialIDFiltersOverride(ID.kFrontLimelightName, Constants.ID.allAprilIDs);
  }
}
