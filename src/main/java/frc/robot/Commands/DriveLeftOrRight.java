package frc.robot.Commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Subsystems.vision.Vision;
import frc.robot.Subsystems.drive.Drivetrain;
import frc.robot.Utils.CoordinateUtilities;

public class DriveLeftOrRight extends DriveDistance {
    Vision vision;
    boolean isLeft;
    double offsetGoal = 0;
    double yError;
    double yOffset = 0;

    public DriveLeftOrRight(Drivetrain dt, Vision vision, boolean isLeft) {
        super(dt);
        this.isLeft = isLeft;
        this.vision = vision;
        this.offsetGoal = 0.17;
        if (isLeft) {
            this.offsetGoal *= -1;
        }
    }

    @Override
    public void initialize() {
        vision.setFiducialIDFilter(0, Constants.ID.reefAprilIDs);

        if (vision.isTargetValid(0)) {
            // TODO: why is this called yDis when it uses X distance?
            double yDis = vision.getTargetDistX(0).in(Units.Meters);
            yError = yDis - offsetGoal;
            try {
                if (isLeft) {
                    this.desiredAngle = 90;
                } else {
                    this.desiredAngle = -90;
                }
                this.desiredDistance = Math.abs(this.yError);
                threshold = 0.007;
                // Set min & max velocity
                minVel = minVelEntry.getDouble(LimelightConstants.minVelocity);
                maxVel = maxVelEntry.getDouble(LimelightConstants.maxVelocity);
                // Get current position from odometry
                currentPose = dt.getRoboPose2d();
                // Get desired position from odometry
                desiredPose = currentPose
                        .plus(CoordinateUtilities.rangeAngleToTransform(this.desiredDistance, this.desiredAngle));
                // Shuffleboard desired pose
                desiredPosXEntry.setValue(desiredPose.getX());
                desiredPosYEntry.setValue(desiredPose.getY());
                // Create a new Trapezoid profile
                profile = new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(maxVel, maxAccEntry.getDouble(LimelightConstants.maxAccMSS)));
            } catch (Exception e) {
                System.out.println("Exception initializing DriveLeftOrRight");
                e.printStackTrace();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        vision.setFiducialIDFilter(0, Constants.ID.allAprilIDs);
    }

}
