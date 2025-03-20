package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.drive.SwerveModule;
import frc.robot.Subsystems.drive.Drivetrain;

public class RotateWheels extends Command {
    protected Drivetrain dt;
    protected double desiredPositionRad;

    public RotateWheels(Drivetrain dt, double desiredAngleDeg) {
        addRequirements(dt);
        this.dt = dt;
        this.desiredPositionRad = Math.toRadians(desiredAngleDeg);
    }

    public void execute() {
        for (SwerveModule sm : dt.getSwerveModules()) {
            sm.goToPosition(desiredPositionRad);
        }
    }

}
