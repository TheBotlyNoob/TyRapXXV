package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Subsystems.drive.Drivetrain;
import frc.robot.Subsystems.vision.Vision;
import frc.robot.Subsystems.vision.VisionIO.TargetObservation;

// Used to get a general location, not exact
public class CenterOnTag extends Command {
    Drivetrain dt;
    Vision vision;

    // Note: xDis, xProportion, xVelocity, xError, and xOffset relate to ySpeed
    // Note: yDis, yProportion, yVelocity, yError, and yOffset relate to xSpeed
    private static GenericEntry rotProportion = Shuffleboard.getTab("Limelight").add("rotProportion", 2).getEntry();
    private static GenericEntry xProportion = Shuffleboard.getTab("Limelight").add("xProportion", 2).getEntry();
    private static GenericEntry yProportion = Shuffleboard.getTab("Limelight").add("yProportion", 2).getEntry();
    private static GenericEntry ySpeedEntry = Shuffleboard.getTab("Limelight").add("ySpeedEntry", 0.0).getEntry();
    private static GenericEntry xSpeedEntry = Shuffleboard.getTab("Limelight").add("xSpeedEntry", 0.0).getEntry();
    private static GenericEntry xDisEntry = Shuffleboard.getTab("Limelight").add("xDisEntry", 0.0).getEntry();
    private static GenericEntry yDisEntry = Shuffleboard.getTab("Limelight").add("yDisEntry", 0.0).getEntry();
    private static GenericEntry xErrorEntry = Shuffleboard.getTab("Limelight").add("xErrorEntry", 0.0).getEntry();
    private static GenericEntry yErrorEntry = Shuffleboard.getTab("Limelight").add("yErrorEntry", 0.0).getEntry();
    private static GenericEntry rotSpeedEntry = Shuffleboard.getTab("Limelight").add("rotSpeedEntry", 0.0).getEntry();
    private static GenericEntry yawAngleEntry = Shuffleboard.getTab("Limelight").add("yawAngleEntry", 0.0).getEntry();
    private static GenericEntry minXVelEntry = Shuffleboard.getTab("Limelight")
            .add("minXVelEntry", LimelightConstants.minXVelocity).getEntry();
    private static GenericEntry maxXVelEntry = Shuffleboard.getTab("Limelight")
            .add("maxXVelEntry", LimelightConstants.maxXVelocity).getEntry();
    private static GenericEntry minYVelEntry = Shuffleboard.getTab("Limelight")
            .add("minYVelEntry", LimelightConstants.minYVelocity).getEntry();
    private static GenericEntry maxYVelEntry = Shuffleboard.getTab("Limelight")
            .add("maxYVelEntry", LimelightConstants.maxYVelocity).getEntry();
    private static GenericEntry minAngVelEntry = Shuffleboard.getTab("Limelight")
            .add("minAngVelEntry", LimelightConstants.minAngVelocityDPS).getEntry();
    private static GenericEntry maxAngVelEntry = Shuffleboard.getTab("Limelight")
            .add("maxAngVelEntry", LimelightConstants.maxAngVelocityDPS).getEntry();
    private static GenericEntry xOffsetEntry = Shuffleboard.getTab("Limelight")
            .add("xOffsetEntry", LimelightConstants.xOffset).getEntry();
    private static GenericEntry yOffsetEntry = Shuffleboard.getTab("Limelight")
            .add("yOffsetEntry", LimelightConstants.yOffset).getEntry();
    private static GenericEntry maxAccEntry = Shuffleboard.getTab("Limelight")
            .add("maxAccEntry", LimelightConstants.maxAccMSS).getEntry();
    private static GenericEntry maxDccEntry = Shuffleboard.getTab("Limelight")
            .add("maxDccEntry", LimelightConstants.maxDccMSS).getEntry();
    private static GenericEntry maxAngAccEntry = Shuffleboard.getTab("Limelight")
            .add("maxAngAccEntry", LimelightConstants.maxAngAccMSS).getEntry();
    private static GenericEntry maxAngDccEntry = Shuffleboard.getTab("Limelight")
            .add("maxAngDccEntry", LimelightConstants.maxAngDccMSS).getEntry();

    public CenterOnTag(Drivetrain dt, Vision vision) {

        this.dt = dt;
        this.vision = vision;
        addRequirements(dt);
    }

    private double xSpeed = 0;
    private double ySpeed = 0;
    private double xError = 0;
    private double yError = 0;
    private double xOffset = 0;
    private double yOffset = 0;
    private double rotSpeed = 0;
    private double minXVel = 0;
    private double maxXVel = 0;
    private double minYVel = 0;
    private double maxYVel = 0;
    private double minAngVel = 0;
    private double maxAngVel = 0;
    private double maxAcc = 0;
    private double maxDcc = 0;
    private double maxAngAcc = 0;
    private double maxAngDcc = 0;
    private double commandedXVel = 0;
    private double commandedYVel = 0;
    private double commandedAngVel = 0;
    private double currentRotProportion = 0.0;
    private double currentXProportion = 0.0;
    private double currentYProportion = 0.0;
    private boolean llLost = false;

    @Override
    public void initialize() {
        // Set FieldRelative to false because we calculate based off of limelight camera
        dt.setFieldRelative(false);

        minXVel = minXVelEntry.getDouble(LimelightConstants.minXVelocity);
        maxXVel = maxXVelEntry.getDouble(LimelightConstants.maxXVelocity);
        minYVel = minYVelEntry.getDouble(LimelightConstants.minYVelocity);
        maxYVel = maxYVelEntry.getDouble(LimelightConstants.maxYVelocity);
        xOffset = xOffsetEntry.getDouble(LimelightConstants.xOffset);
        yOffset = yOffsetEntry.getDouble(LimelightConstants.yOffset);
        minAngVel = minAngVelEntry.getDouble(LimelightConstants.minAngVelocityDPS);
        maxAngVel = maxAngVelEntry.getDouble(LimelightConstants.maxAngVelocityDPS);
        maxAcc = maxAccEntry.getDouble(LimelightConstants.maxAccMSS) / (1 / 0.02);
        maxDcc = maxDccEntry.getDouble(LimelightConstants.maxDccMSS) / (1 / 0.02);
        maxAngAcc = maxAngAccEntry.getDouble(LimelightConstants.maxAngAccMSS) / (1 / 0.02);
        maxAngDcc = maxAngDccEntry.getDouble(LimelightConstants.maxAngDccMSS) / (1 / 0.02);
        currentRotProportion = rotProportion.getDouble(2);
        currentXProportion = xProportion.getDouble(2.0);
        currentYProportion = yProportion.getDouble(2.0);
        llLost = false;

        System.out.println("minVel=" + minXVel + " minAngVel=" + minAngVel + "  maxAngVel=" + maxAngVel);
    }

    @Override
    public void execute() {
        // Check if we have a valid target before calculating and setting values
        if (vision.isTargetValid(0)) {
            Rotation2d rotAngle = vision.getTargetYaw(0);
            double xDis = vision.getTargetDistX(0).in(Units.Meters);
            double yDis = vision.getTargetDistY(0).in(Units.Meters);

            xError = xDis - xOffset;
            yError = yDis - yOffset;
            double calculatedXVel = Math.abs(xError * currentXProportion);
            double calculatedYVel = Math.abs(yError * currentYProportion);
            double calculatedAngVel = Math.abs(rotAngle.getDegrees() * currentRotProportion);
            double desiredXVel;
            double desiredYVel;
            double desiredAngVel;
            double deltaVel;

            // Desired Velocities
            if (Math.abs(xError) < LimelightConstants.xDisThreshold) {
                desiredXVel = 0;
            } else {
                desiredXVel = Math.copySign(MathUtil.clamp(calculatedXVel, minXVel, maxXVel), xError);
            }
            // if (Math.abs(yError) < LimelightConstants.yDisThreshold) {
            // desiredYVel = 0;
            // } else {
            // desiredYVel = Math.copySign(MathUtil.clamp(calculatedYVel, minYVel, maxYVel),
            // yError);
            // }
            // if (Math.abs(ll.getYawAngleDegrees()) < LimelightConstants.rotThreshold) {
            // desiredAngVel = 0;
            // } else {
            // desiredAngVel = Math.copySign(MathUtil.clamp(calculatedAngVel, minAngVel,
            // maxAngVel), rotAngleDegrees);
            // }

            // Commanded X Velocity ramped
            if ((Math.abs(desiredXVel) - Math.abs(commandedXVel)) > 0) {
                deltaVel = maxAcc;
            } else {
                deltaVel = maxDcc;
            }
            if (desiredXVel > commandedXVel) {
                commandedXVel = Math.min(desiredXVel, commandedXVel + deltaVel);
            } else if (desiredXVel < commandedXVel) {
                commandedXVel = Math.max(desiredXVel, commandedXVel - deltaVel);
            }

            // // Commanded Y Velocity ramped
            // if ((Math.abs(desiredYVel) - Math.abs(commandedYVel)) > 0) {
            // deltaVel = maxAcc;
            // } else {
            // deltaVel = maxDcc;
            // }
            // if (desiredYVel > commandedYVel) {
            // commandedYVel = Math.min(desiredYVel, commandedYVel + deltaVel);
            // } else if (desiredYVel < commandedYVel) {
            // commandedYVel = Math.max(desiredYVel, commandedYVel - deltaVel);
            // }

            // // Commanded angular Velocity ramped
            // if ((Math.abs(desiredAngVel) - Math.abs(commandedAngVel)) > 0) {
            // deltaVel = maxAngAcc;
            // } else {
            // deltaVel = maxAngDcc;
            // }
            // if (desiredAngVel > commandedAngVel) {
            // commandedAngVel = Math.min(desiredAngVel, commandedAngVel + deltaVel);
            // } else if (desiredAngVel < commandedAngVel) {
            // commandedAngVel = Math.max(desiredAngVel, commandedAngVel - deltaVel);
            // }

            ySpeed = commandedXVel;
            xSpeed = commandedYVel;
            rotSpeed = commandedAngVel;
            ySpeedEntry.setDouble(ySpeed);
            xSpeedEntry.setDouble(xSpeed);
            rotSpeedEntry.setDouble(rotSpeed);
            xDisEntry.setDouble(xDis);
            yDisEntry.setDouble(yDis);
            xErrorEntry.setDouble(xError);
            yErrorEntry.setDouble(yError);
            xOffsetEntry.setDouble(xOffset);
            yOffsetEntry.setDouble(yOffset);
            yawAngleEntry.setDouble(rotAngle.getDegrees());
            dt.drive(xSpeed, ySpeed, Math.toRadians(rotSpeed));
        } else {
            llLost = true;
        }

    }
}

// @Override
// public boolean isFinished() {
// // if (Math.abs(xError) < LimelightConstants.xDisThreshold && Math
// // //.abs(yError) < LimelightConstants.yDisThreshold
// // && Math.abs(ll.getFilteredYawDegrees()) < LimelightConstants.rotThreshold)
// {
// // // Set FieldRelative back to true so the Drivetrain works for teleop
// // dt.setFieldRelative(true);
// // dt.drive(0, 0, 0);
// // System.out.println("COT command complete");
// // return true;
// // } else if (llLost == true) {
// // System.out.println("COT command aborted because limelight lost");
// // // Set FieldRelative back to true so the Drivetrain works for teleop
// // dt.setFieldRelative(true);
// // dt.drive(0, 0, 0);
// // return true;
// // }
// // return false;
// }
// }
