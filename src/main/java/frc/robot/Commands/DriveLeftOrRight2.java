package frc.robot.Commands;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.ID;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Limelight;
import frc.robot.Utils.CoordinateUtilities;
import frc.robot.Utils.TrapezoidController;

public class DriveLeftOrRight2 extends DriveDistance2 {
  Limelight ll;
  boolean isLeft;
  double offsetGoal = 0;
  double yError;
  double yOffset = 0;

  public DriveLeftOrRight2(Drivetrain dt, Limelight ll, boolean isLeft) {
    super(dt);
    this.isLeft = isLeft;
    this.ll = ll;
    this.offsetGoal = 0.17;
    if (isLeft) {
      this.offsetGoal *= -1;
    }
  }

  @Override
  public void initialize() {
    LimelightHelpers.SetFiducialIDFiltersOverride(
        ID.kFrontLimelightName, Constants.ID.reefAprilIDs);
    double yDis = -1 * ll.getxDistanceMeters();
    yError = yDis - offsetGoal;
    try {
      if (yError > 0) {
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
      desiredPose =
          currentPose.plus(
              CoordinateUtilities.rangeAngleToTransform(this.desiredDistance, this.desiredAngle));
      // Shuffleboard desired pose
      desiredPosXEntry.setValue(desiredPose.getX());
      desiredPosYEntry.setValue(desiredPose.getY());

      // Calculate proportion of current velocity in line with the desired velocity
      chassisSpeed = dt.getChassisSpeeds();
      var currentSpeedVec = new Vector<>(Nat.N2());
      currentSpeedVec.set(0, 0, chassisSpeed.vxMetersPerSecond);
      currentSpeedVec.set(1, 0, chassisSpeed.vyMetersPerSecond);

      var offsetVector = new Vector<>(Nat.N2());
      offsetVector.set(0, 0, desiredPose.getX());
      offsetVector.set(1, 0, desiredPose.getY());
      var offsetUnitVector = offsetVector.div(offsetVector.norm());
      double speedInDesiredDirection = currentSpeedVec.dot(offsetUnitVector);

      // Create a new Trapezoid profile
      profile =
          new TrapezoidController(
              speedInDesiredDirection,
              threshold,
              minVel,
              maxVel,
              maxAccEntry.getDouble(LimelightConstants.driveOffsetMaxAccMSS),
              10,
              decelKpEntry.getDouble(4.0));
    } catch (Exception e) {
      System.out.println("Exception initializing DriveLeftOrRight");
      e.printStackTrace();
    }
    LimelightHelpers.SetFiducialIDFiltersOverride(ID.kFrontLimelightName, Constants.ID.allAprilIDs);
  }

  @Override
  public void execute() {
    // Get current time
    double lastTime = Timer.getFPGATimestamp();
    // Calculate distance to the new position from the current one
    double yDis = -1 * ll.getxDistanceMeters();
    yError = yDis - offsetGoal;
    rangeM = Math.abs(yError);
    if (yError > 0) {
      this.desiredAngle = 90;
    } else {
      this.desiredAngle = -90;
    }
    // Calculate bearing
    bearingDeg = this.desiredAngle;
    // Get chassis speed
    chassisSpeed = dt.getChassisSpeeds();
    // Get chassis magnitude
    chassisMagnitude = CoordinateUtilities.getChassisMagnitude(chassisSpeed);
    // Set Trapezoid profile
    var setpoint = profile.calculate(rangeM, chassisMagnitude);
    currentVel = Math.max(setpoint, minVel);
    // Calculate x and y components of desired velocity
    calcVel = CoordinateUtilities.courseSpeedToLinearVelocity(bearingDeg, currentVel);
    // Drive
    dt.driveChassisSpeeds(calcVel);
    // Set range, bearing, vel, and time in Shuffleboard
    rangeMEntry.setDouble(rangeM);
    bearingEntry.setDouble(bearingDeg);
    currentXVelEntry.setDouble(calcVel.vxMetersPerSecond);
    currentYVelEntry.setDouble(calcVel.vyMetersPerSecond);
    timeEntry.setDouble(lastTime);
  }

  @Override
  public boolean isFinished() {
    if (rangeM <= threshold) {
      return true;
    }
    return false;
  }
}
