package frc.sim;

import java.util.stream.Stream;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import frc.robot.Subsystems.drive.Drivetrain;
import frc.robot.Subsystems.drive.GyroIOPigeon2;
import frc.robot.Subsystems.drive.SwerveModuleIO;

public class SimDrivetrain extends Drivetrain {

    public SimDrivetrain(Pigeon2 gyro, SwerveModuleIO frontLeft, SwerveModuleIO frontRight, SwerveModuleIO backLeft,
            SwerveModuleIO backRight) {
        super(new GyroIOPigeon2(gyro), frontLeft, frontRight, backLeft, backRight);
    }

    protected float desiredXSpeedMps = 0;
    protected float desiredYSpeedMps = 0;
    protected float desiredRotationSpeedDps = 0;
    protected float currentXSpeedMps = 0;
    protected float currentYSPeedMps = 0;
    protected float currentRotSpeedDps = 0;

    protected Pose3d currentPose = new Pose3d();

    protected final double loopTime = 0.02;

    // TODO: use real robot values

    /*
     * public SimDrivetrain() {
     * super(new Pigeon2(0));
     * 
     * SimulatedArena.getInstance().addDriveTrainSimulation(sim);
     * }
     * 
     * @Override
     * public SwerveModulePosition[] getModulePositions() {
     * return (SwerveModulePosition[]) Stream.of(sim.getModules())
     * .map((m) -> new
     * SwerveModulePosition(m.getDriveWheelFinalPosition().in(Units.Radians),
     * new Rotation2d(m.getSteerAbsoluteAngle())))
     * .toArray();
     * }
     */
    public Pose3d getSimPose() {
        return currentPose;
    }

    public void setSimPose(Pose3d pose) {
        currentPose = pose;
    }

}
