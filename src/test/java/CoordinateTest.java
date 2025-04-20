import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Utils.CoordinateUtilities;

public class CoordinateTest {

  @org.junit.jupiter.api.Test
  void coordinateTest() {
    // Get the current position
    Pose2d currentPos = new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(45.0)));
    System.out.println("currentPos: " + currentPos);
    System.out.println(
        "Forward 1m: " + currentPos.plus(CoordinateUtilities.rangeAngleToTransform(1.0, 0.0)));
    System.out.println(
        "Right 1m:" + currentPos.plus(CoordinateUtilities.rangeAngleToTransform(1.0, -90.0)));
    System.out.println(
        "Left 1m:" + currentPos.plus(CoordinateUtilities.rangeAngleToTransform(1.0, 90.0)));

    // Starting at 0 degree heading
    currentPos = new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(0.0)));
    System.out.println(
        "Forward 1m: " + currentPos.plus(CoordinateUtilities.rangeAngleToTransform(1.0, 0.0)));
    System.out.println(
        "Right 1m:" + currentPos.plus(CoordinateUtilities.rangeAngleToTransform(1.0, -90.0)));
    System.out.println(
        "Left 1m:" + currentPos.plus(CoordinateUtilities.rangeAngleToTransform(1.0, 90.0)));

    // Starting at 90 degree heading
    currentPos = new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(90.0)));
    System.out.println(
        "Forward 1m: " + currentPos.plus(CoordinateUtilities.rangeAngleToTransform(1.0, 0.0)));
    System.out.println(
        "Right 1m:" + currentPos.plus(CoordinateUtilities.rangeAngleToTransform(1.0, -90.0)));
    System.out.println(
        "Left 1m:" + currentPos.plus(CoordinateUtilities.rangeAngleToTransform(1.0, 90.0)));

    // Convert course and speed
    ChassisSpeeds chassisSpeeds = CoordinateUtilities.courseSpeedToLinearVelocity(45.0, 2.0);
    System.out.println("Chassis speeds from 45deg at 2mps: " + chassisSpeeds);

    // Calculate distance
    double rangeM =
        CoordinateUtilities.distanceTo(new Pose2d(), new Pose2d(3.0, 4.0, new Rotation2d()));
    System.out.println("Range from origin to 3, 4: " + rangeM);

    // Calculate bearing
    double bearingDeg =
        CoordinateUtilities.bearingTo(new Pose2d(), new Pose2d(10.0, 5.0, new Rotation2d()));
    System.out.println("Bearing origin to 10, 5: " + bearingDeg);

    bearingDeg =
        CoordinateUtilities.bearingTo(new Pose2d(), new Pose2d(-10.0, 5.0, new Rotation2d()));
    System.out.println("Bearing origin to -10, 5: " + bearingDeg);

    bearingDeg =
        CoordinateUtilities.bearingTo(new Pose2d(), new Pose2d(-10.0, -5.0, new Rotation2d()));
    System.out.println("Bearing origin to -10, -5: " + bearingDeg);

    Pose2d tagPose = new Pose2d(3.0, 2.0, new Rotation2d(Math.toRadians(225.0)));
    Transform2d desiredOffset = new Transform2d(0.5, -0.1, new Rotation2d());
    Pose2d desiredEndPoseRobotRelative = tagPose.plus(desiredOffset);
    Pose2d currentRobotPose = new Pose2d(1.0, 1.0, new Rotation2d(Math.toRadians(90.0)));
    System.out.println("tagPose: " + tagPose);
    System.out.println("desiredOffset: " + desiredOffset);
    System.out.println("desiredEndPoseRobotRelative: " + desiredEndPoseRobotRelative);
    Pose2d desiredEndPoseFieldRelative =
        currentRobotPose.plus(
            new Transform2d(
                desiredEndPoseRobotRelative.getX(),
                desiredEndPoseRobotRelative.getY(),
                new Rotation2d()));
    System.out.println("currentRobotPose: " + currentRobotPose);
    System.out.println("desiredEndPoseFieldRelative: " + desiredEndPoseFieldRelative);

    assertTrue(true);
  }
}
