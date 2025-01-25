package frc.sim;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystems.Drivetrain;

public class SimDrivetrain extends Drivetrain {

    protected float desiredXSpeedMps = 0;
    protected float desiredYSpeedMps = 0;
    protected float desiredRotationSpeedDps = 0;
    protected float currentXSpeedMps = 0;
    protected float currentYSPeedMps = 0;
    protected float currentRotSpeedDps = 0;

    protected Pose3d currentPose = new Pose3d();

    protected final double loopTime = 0.02;
    
    public SimDrivetrain()
    {
        super(new Pigeon2(0));
    }

    @Override
    public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        // Updated the simulated positions
        this.desiredXSpeedMps = (float)chassisSpeeds.vxMetersPerSecond;
        this.desiredYSpeedMps = (float)chassisSpeeds.vyMetersPerSecond;
        this.desiredRotationSpeedDps = (float)Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond);
    }

    @Override
    public void simulationPeriodic() {
        // Update position and velocity
        this.currentXSpeedMps = desiredXSpeedMps;
        this.currentYSPeedMps = desiredYSpeedMps;
        this.currentRotSpeedDps = desiredRotationSpeedDps;

        this.currentPose = new Pose3d(currentPose.getX() + currentXSpeedMps*loopTime,
            currentPose.getY() + currentYSPeedMps*loopTime, 0.0,
            new Rotation3d(0.0, 0.0, currentPose.getRotation().getZ() + Math.toRadians(currentRotSpeedDps*loopTime)));  
    }

    public Pose3d getSimPose() {
        return this.currentPose;
    }

    public void setSimPose(Pose3d newPose) {
        this.currentPose = newPose;
    }
}
