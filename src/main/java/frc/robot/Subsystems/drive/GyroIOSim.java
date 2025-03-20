package frc.robot.Subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation;

    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yawPosition = gyroSimulation.getGyroReading();
        inputs.yawVelocity = gyroSimulation.getMeasuredAngularVelocity();
    }

    @Override
    public boolean setYaw(Rotation2d yaw) {
        gyroSimulation.setRotation(yaw);
        return true;
    }
}
