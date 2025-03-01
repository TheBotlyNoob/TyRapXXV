package frc.robot.Commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Limelight;
import frc.robot.Utils.CoordinateUtilities;

public class DriveLeftOrRight extends DriveDistance {
    Limelight ll;
    boolean isLeft;
    double offsetGoal = 0;
    double yError;
    double yOffset = 0;
    public DriveLeftOrRight(Drivetrain dt, Limelight ll, boolean isLeft){
        super(dt);
        this.isLeft = isLeft;
        this.ll = ll;
        this.offsetGoal = 0.165;
        if (isLeft){
            this.offsetGoal *= -1;
        }
    }
    @Override
    public void initialize() {
        if (ll.getTimeSinceValid() == 0) {
            double yDis = -1 * ll.getxDistanceMeters();
            yError = yDis - offsetGoal;
            try{
                if (isLeft){
                    this.desiredAngle = 90;
                } else {this.desiredAngle = -90;}
                this.desiredDistance = Math.abs(this.yError);
                threshold = 0.005;
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
                System.out.println("Exception initializing DriveDistance");
                e.printStackTrace();
            }
        }
        
    }
    
}
