package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.TyRap24Constants.*;
import frc.robot.SparkJrConstants.*;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Limelight;

public class CenterOnTag extends Command {
    Drivetrain dt;
    Limelight ll;
    
    public CenterOnTag(Drivetrain dt, Limelight ll) {
        this.dt = dt;
        this.ll = ll;
        addRequirements(dt);
    }

    private double xSpeed = 0;
    private double ySpeed = 0;
    private double rotSpeed = 0;



    @Override
    public void execute() {
        double xDis = ll.getxDistanceMeters();
        ySpeed = -1*Math.min(1.5, xDis * 2);
        dt.drive(xSpeed, ySpeed, rotSpeed);
    }

    @Override
    public boolean isFinished() {
        if(Math.abs(ll.getxDistanceMeters())<0.05){
            return true;
        }
        return false;
    }
}
