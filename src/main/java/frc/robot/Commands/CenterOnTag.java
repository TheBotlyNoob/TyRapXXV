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
    private GenericEntry rotProportion = Shuffleboard.getTab("Limelight").add("rotProportion", 0.5).getEntry();
    private GenericEntry xProportion = Shuffleboard.getTab("Limelight").add("xProportion", 0.5).getEntry();
    private GenericEntry ySpeedEntry = Shuffleboard.getTab("Limelight").add("ySpeedEntry", 0.5).getEntry();
    private GenericEntry xDisEntry = Shuffleboard.getTab("Limelight").add("xDisEntry", 0.5).getEntry();
    private GenericEntry rotSpeedEntry = Shuffleboard.getTab("Limelight").add("rotSpeedEntry", 0.5).getEntry();
    private GenericEntry yawAnglEntry = Shuffleboard.getTab("Limelight").add("yawAngleEntry", 0.5).getEntry();
    private GenericEntry minVelEntry = Shuffleboard.getTab("Limelight").add("minVelEntry", LimelightConstants.minLinearVelocity).getEntry();

    public CenterOnTag(Drivetrain dt, Limelight ll) {

        this.dt = dt;
        this.ll = ll;
        addRequirements(dt);
    }

    private double xSpeed = 0;
    private double ySpeed = 0;
    private double rotSpeed = 0;

    @Override
    public void initialize() {
        dt.setFieldRelative(false);
    }

    @Override
    public void execute() {
        double currentRotProportion = rotProportion.getDouble(0.5);
        double rotAngleDegrees = -1 * ll.getFilteredYawDegrees();
        double xDis = -1 * ll.getxDistanceMeters();
        rotSpeed = Math.copySign(Math.min(1.2, Math.abs(rotAngleDegrees * currentRotProportion)), rotAngleDegrees);
        double desiredVel = Math.abs(xDis * xProportion.getDouble(1.0));
        double minimum = minVelEntry.getDouble(LimelightConstants.minLinearVelocity);
        ySpeed = Math.copySign(MathUtil.clamp(desiredVel, minimum, 1.0), xDis);
        ySpeedEntry.setDouble(ySpeed);
        xDisEntry.setDouble(xDis);
        rotSpeedEntry.setDouble(rotSpeed);
        yawAnglEntry.setDouble(rotAngleDegrees);
        dt.drive(xSpeed, ySpeed, Math.toRadians(rotSpeed));
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(ll.getxDistanceMeters()) < 0.05 && Math.abs(ll.getYawAngleDegrees()) < 2) {
            dt.setFieldRelative(true);
            dt.drive(0,0,0);
            System.out.println("COT command complete");
            return true;
        }
        return false;
    }
}
