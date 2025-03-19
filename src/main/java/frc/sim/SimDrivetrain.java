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

    // TODO: use real robot values
    protected final DriveTrainSimulationConfig simConf = DriveTrainSimulationConfig.Default().withGyro(COTS.ofPigeon2())
            .withSwerveModule(new SwerveModuleSimulationConfig(
                    DCMotor.getNeoVortex(1), // drive motor
                    DCMotor.getNeoVortex(1), // steer motor
                    6.12, // drive motor gear ratio
                    12.8, // steer motor gear ratio
                    Units.Volts.of(0.1), // drive friction, in voltage
                    Units.Volts.of(0.1), // steer friction, in voltage
                    Units.Inches.of(2), // wheel radius
                    Units.KilogramSquareMeters.of(0.03), // steer rotational inertia
                    1.2 // wheel coefficient of friction
            )).withBumperSize(Units.Inches.of(30), Units.Inches.of(30))
            .withTrackLengthTrackWidth(Units.Inches.of(24), Units.Inches.of(24));

    protected final SwerveDriveSimulation sim = new SwerveDriveSimulation(simConf, new Pose2d(3, 3, new Rotation2d()));

    public SimDrivetrain() {
        super(new Pigeon2(0));

        SimulatedArena.getInstance().addDriveTrainSimulation(sim);
    }

    @Override
    public SwerveModulePosition[] getModulePositions() {
        return (SwerveModulePosition[]) Stream.of(sim.getModules())
                .map((m) -> new SwerveModulePosition(m.getDriveWheelFinalPosition().in(Units.Radians),
                        new Rotation2d(m.getSteerAbsoluteAngle())))
                .toArray();
    }
}
