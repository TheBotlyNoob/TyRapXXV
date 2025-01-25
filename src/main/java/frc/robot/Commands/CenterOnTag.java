package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
//import frc.robot.TyRap24Constants.*;
import frc.robot.SparkJrConstants.*;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Limelight;

public class CenterOnTag extends Command {
    Drivetrain dt;
    Limelight ll;
    // Note: xDis, xProportion, and xVelocity relate to ySpeed
    // Note: yDis and yProportion relate to xSpeed
    private static GenericEntry rotProportion = Shuffleboard.getTab("Limelight").add("rotProportion", 2).getEntry();
    private static GenericEntry xProportion = Shuffleboard.getTab("Limelight").add("xProportion", 2).getEntry();
    private static GenericEntry ySpeedEntry = Shuffleboard.getTab("Limelight").add("ySpeedEntry", 0.5).getEntry();
    private static GenericEntry xDisEntry = Shuffleboard.getTab("Limelight").add("xDisEntry", 0.5).getEntry();
    private static GenericEntry rotSpeedEntry = Shuffleboard.getTab("Limelight").add("rotSpeedEntry", 0.5).getEntry();
    private static GenericEntry yawAngleEntry = Shuffleboard.getTab("Limelight").add("yawAngleEntry", 0.5).getEntry();
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

    public CenterOnTag(Drivetrain dt, Limelight ll) {

        this.dt = dt;
        this.ll = ll;
        addRequirements(dt);
    }

    private double xSpeed = 0;
    private double ySpeed = 0;
    private double rotSpeed = 0;
    private double minXVel = 0;
    private double maxXVel = 0;
    private double minAngVel = 0;
    private double maxAngVel = 0;
    private double currentRotProportion = 0.0;
    private double currentXProportion = 0.0;
    private boolean llLost = false;

    @Override
    public void initialize() {
        dt.setFieldRelative(false);
        minXVel = minXVelEntry.getDouble(LimelightConstants.minXVelocity);
        maxXVel = maxXVelEntry.getDouble(LimelightConstants.maxXVelocity);
        minAngVel = minAngVelEntry.getDouble(LimelightConstants.minAngVelocityDPS);
        maxAngVel = maxAngVelEntry.getDouble(LimelightConstants.maxAngVelocityDPS);
        currentRotProportion = rotProportion.getDouble(2);
        currentXProportion = xProportion.getDouble(1.0);
        llLost = false;
        System.out.println("minVel=" + minXVel + " minAngVel=" + minAngVel + "  maxAngVel=" + maxAngVel);
    }

    @Override
    public void execute() {
        if (ll.getTimeSinceValid() == 0) {
            double rotAngleDegrees = -1 * ll.getFilteredYawDegrees();
            double xDis = -1 * ll.getxDistanceMeters();
            double desiredVel = Math.abs(xDis * currentXProportion);
            double desiredAngVel = Math.abs(rotAngleDegrees * currentRotProportion);
            if (Math.abs(ll.getxDistanceMeters()) < LimelightConstants.xDisThreshold) {
                ySpeed = 0;
            } else {
                ySpeed = Math.copySign(MathUtil.clamp(desiredVel, minXVel, maxXVel), xDis);
            }
            if (Math.abs(ll.getYawAngleDegrees()) < LimelightConstants.rotThreshold) {
                rotSpeed = 0;
            } else {
                rotSpeed = Math.copySign(MathUtil.clamp(desiredAngVel, minAngVel, maxAngVel), rotAngleDegrees);
            }
            ySpeedEntry.setDouble(ySpeed);
            rotSpeedEntry.setDouble(rotSpeed);
            xDisEntry.setDouble(xDis);
            yawAngleEntry.setDouble(rotAngleDegrees);
            dt.drive(xSpeed, ySpeed, Math.toRadians(rotSpeed));
        } else if (ll.getTimeSinceValid() < 10) {

        } else {
            llLost = true;
        }

    }

    @Override
    public boolean isFinished() {
        if (Math.abs(ll.getxDistanceMeters()) < LimelightConstants.xDisThreshold
                && Math.abs(ll.getYawAngleDegrees()) < LimelightConstants.rotThreshold) {
            dt.setFieldRelative(true);
            dt.drive(0, 0, 0);
            System.out.println("COT command complete");
            return true;
        } else if (llLost == true) {
            System.out.println("COT command aborted because limelight lost");
            dt.setFieldRelative(true);
            dt.drive(0, 0, 0);
            return true;
        }
        return false;
    }
}
