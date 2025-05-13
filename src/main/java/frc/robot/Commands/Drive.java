package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.Subsystems.Drivetrain;

public class Drive extends Command {
  Drivetrain dt;
  private final XboxController m_controller = new XboxController(Controller.kDriveControllerID);
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(Controller.kRateLimitXSpeed);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(Controller.kRateLimitYSpeed);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Controller.kRateLimitRot);

  public Drive(Drivetrain dt) {
    this.dt = dt;
    addRequirements(dt);
  }

  private double xSpeed;
  private double ySpeed;
  private double rotSpeed;

  private void readControllers() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    xSpeed =
        -m_xspeedLimiter.calculate(
                MathUtil.applyDeadband(m_controller.getLeftY(), Deadbands.kLeftJoystickDeadband))
            * Controller.kMaxNecessarySpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    ySpeed =
        -m_yspeedLimiter.calculate(
                MathUtil.applyDeadband(m_controller.getLeftX(), Deadbands.kLeftJoystickDeadband))
            * Controller.kMaxNecessarySpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    rotSpeed =
        -m_rotLimiter.calculate(
                MathUtil.applyDeadband(m_controller.getRightX(), Deadbands.kRightJoyStickDeadband))
            * Drivetrain.kMaxAngularSpeed;

    if (m_controller.getLeftStickButton()) {
      xSpeed *= .25;
      ySpeed *= .25;
      rotSpeed *= .25;
    }
  }

  @Override
  public void execute() {
    readControllers();
    dt.drive(xSpeed, ySpeed, rotSpeed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
