package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Controller;
import frc.robot.Subsystems.drive.Drivetrain;

public class DriveFixedVelocity extends Command {
    Drivetrain dt;
    int pov;
    DoubleSupplier speedSupplier; 
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(Controller.kRateLimitXSpeed);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(Controller.kRateLimitYSpeed);

    public DriveFixedVelocity(Drivetrain dt, int pov, DoubleSupplier speedSupplier) {
        this.pov = pov;
        this.speedSupplier = speedSupplier;
        this.dt = dt;
        addRequirements(dt);
    }

    private double xSpeed;
    private double ySpeed;    

    public void initialize() {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        switch (pov) {
            case 0:
                xSpeed = speedSupplier.getAsDouble();
                ySpeed = 0;
                break;
            case 90:
                xSpeed = 0;
                ySpeed = -speedSupplier.getAsDouble();
                break;
            case 180:
                xSpeed = -speedSupplier.getAsDouble();
                ySpeed = 0;
                break;
            case 270:
                xSpeed = 0;
                ySpeed = speedSupplier.getAsDouble();
        }
    }

    @Override
    public void execute() {
        double actualXSpeed = m_xspeedLimiter
                .calculate(xSpeed);
        double actualYSpeed = m_yspeedLimiter
                .calculate(ySpeed);
        dt.driveChassisSpeeds(new ChassisSpeeds(actualXSpeed, actualYSpeed, 0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
