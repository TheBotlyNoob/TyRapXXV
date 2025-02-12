package frc.robot.Commands;

// Imports
import frc.robot.Subsystems.Drivetrain;
import java.util.function.DoubleSupplier;

// This Command uses DriveDistance Command and the range sensor to drive until the range is a certain value or has gone too far
public class DriveRange extends DriveDistance {
    private double rangeThreshold;
    private DoubleSupplier rangeSupplier;
    
    // Constructors invoking DriveDistance
    public DriveRange(Drivetrain dt, DoubleSupplier maxDistanceSupplier, DoubleSupplier rangeSupplier, double driveAngle, double rangeThreshold){
        super(dt, maxDistanceSupplier, driveAngle);
        this.rangeSupplier = rangeSupplier;
        this.rangeThreshold = rangeThreshold;
    }

    public DriveRange(Drivetrain dt) {
        super(dt);
    }
    
    @Override
    public boolean isFinished() {
        System.out.println("Range val: "+rangeSupplier.getAsDouble());
        if (rangeSupplier.getAsDouble() >= rangeThreshold || super.isFinished()) {
            return true;
        }
        return false;
    }
}
