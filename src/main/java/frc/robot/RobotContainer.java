// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.*;
import frc.robot.Constants.*;
import frc.robot.Constants.RobotMode.Mode;
import frc.robot.Subsystems.LightSubsystem;
import frc.robot.Subsystems.algae.*;
import frc.robot.Subsystems.climber.*;
import frc.robot.Subsystems.coral.*;
import frc.robot.Subsystems.drive.*;
import frc.robot.Subsystems.elevator.*;
import frc.robot.Subsystems.vision.Vision;
import frc.robot.Subsystems.vision.VisionIO;
import frc.robot.Subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.Subsystems.vision.limelight.VisionIOLimelight;
import frc.robot.Utils.SafeableSubsystem;

import org.dyn4j.geometry.Triangle;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public final NetworkTableInstance nt = NetworkTableInstance.getDefault();

    private Drivetrain m_swerve;
    private Vision m_vision;
    private AlgaeGrabberSubsystem m_algae;
    private ClimberSubsystem m_climber;
    private ElevatorSubsystem m_elevator;
    private CoralSubsystem m_coral;
    private LightSubsystem m_leds;

    private final LoggedDashboardChooser<String> autoChooser;

    protected UsbCamera climbCamera;

    Command m_driveCommand;

    boolean bindingsConfigured = false;

    SequentialCommandGroup m_scoreCancel;
    SequentialCommandGroup m_scoreLeft;
    SequentialCommandGroup m_scoreRight;

    Optional<SwerveDriveSimulation> driveSim = Optional.empty();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.RobotMode.currentMode) {
            case REAL:
                initReal();
                break;
            case SIM:
                initSim();
                break;
            case REPLAY:
            default:
                initReplay();
                break;

        }

        climbCamera = CameraServer.startAutomaticCapture(); // Start USB webcam capture for climb
        climbCamera.setFPS(18);
        climbCamera.setPixelFormat(PixelFormat.kMJPEG);

        // this.m_range = new RangeSensor(0);

        AddressableLED led = new AddressableLED(0);
        led.setLength(5);
        AddressableLEDBuffer ledBuf = new AddressableLEDBuffer(5);

        this.m_leds = new LightSubsystem(led, ledBuf, m_vision, m_coral, m_elevator);

        // Xbox controllers return negative values when we push forward.
        this.m_driveCommand = new Drive(m_swerve);
        this.m_swerve.setDefaultCommand(this.m_driveCommand);

        autoChooser = new LoggedDashboardChooser<>("AutoRoutine"); // Default auto will be
                                                                   // `Commands.none()'

        configurePathPlanner();
        autoChooser.addDefaultOption("DO NOTHING!", "NO AUTO");

        // configureBindings();
        NamedCommands.registerCommand("StopDrive", new StopDrive(m_swerve));

        this.m_scoreLeft = new SequentialCommandGroup(
                new ConditionalCommand(
                        new ConditionalCommand(
                                buildScoreOffsetCommand(true),
                                buildScoreBumperedUpCommand(true, 0.15),
                                () -> m_vision
                                        .getTargetDistZ(0).in(Units.Meters) > (Offsets.cameraOffsetFromFrontBumber
                                                + 0.1)),
                        new PrintCommand("level has not been set")
                                .andThen(new RumbleController(
                                        Controller.kManipulatorController
                                                .getHID(),
                                        .5)),
                        () -> (m_elevator.isAnyLevelSet()) && m_vision.isTargetValid(0) && Constants.ID.reefAprilIDs
                                .contains(m_vision.getFiducialID(0))));

        this.m_scoreRight = new SequentialCommandGroup(
                new ConditionalCommand(
                        new ConditionalCommand(
                                buildScoreOffsetCommand(false),
                                buildScoreBumperedUpCommand(false, 0.15),
                                () -> m_vision
                                        .getTargetDistZ(0).in(Units.Meters) > (Offsets.cameraOffsetFromFrontBumber
                                                + 0.1)),
                        new PrintCommand("level has not been set").andThen(new RumbleController(
                                Controller.kManipulatorController.getHID(), .5)),
                        () -> (m_elevator.isAnyLevelSet()) && m_vision.isTargetValid(0) && Constants.ID.reefAprilIDs
                                .contains(m_vision.getFiducialID(0))));

        this.m_scoreCancel = new SequentialCommandGroup(
                m_elevator.runOnce(() -> m_scoreLeft.cancel()),
                m_elevator.runOnce(() -> m_scoreRight.cancel()),
                new StopDrive(m_swerve),
                new StopCoral(m_coral),
                new StopElevator(m_elevator));
    }

    private void initReal() throws IllegalStateException {
        if (Constants.RobotMode.currentMode != Mode.REAL) {
            throw new IllegalStateException("initReal can only be called in REAL mode");
        }

        m_swerve = new Drivetrain(new GyroIOPigeon2(new Pigeon2(Constants.ID.kGyro)),
                new SwerveModuleIOSpark(ID.kFrontLeftDrive, ID.kFrontLeftTurn, ID.kFrontLeftCANCoder,
                        Offsets.kFrontLeftOffset,
                        DrivetrainConstants.sparkFlex, true),

                new SwerveModuleIOSpark(ID.kFrontRightDrive, ID.kFrontRightTurn, ID.kFrontRightCANCoder,
                        Offsets.kBackRightOffset,
                        DrivetrainConstants.sparkFlex, false),

                new SwerveModuleIOSpark(ID.kBackLeftDrive, ID.kBackLeftTurn, ID.kBackLeftCANCoder,
                        Offsets.kBackLeftOffset,
                        DrivetrainConstants.sparkFlex, false),

                new SwerveModuleIOSpark(ID.kBackRightDrive, ID.kBackRightTurn, ID.kBackRightCANCoder,
                        Offsets.kBackLeftOffset,
                        DrivetrainConstants.sparkFlex, false));

        m_vision = new Vision(m_swerve::addVisionMeasurement,
                Constants.ID.allAprilIDs,
                new VisionIOLimelight(m_swerve::getGyroYawRotation2d));

        m_algae = new AlgaeGrabberSubsystem(new AlgaePneumaticsIOReal(), new AlgaeRetrievalIOSpark());

        m_elevator = new ElevatorSubsystem(
                new ElevatorMotorIOSpark(),
                new ElevatorLimitsIOReal(),
                new ElevatorConfigIONetworkTables(nt));

        m_coral = new CoralSubsystem(
                m_elevator,
                new CoralGrabberIOSpark(),
                new CoralWristIOSpark(),
                new CoralDetectionIOReal(),
                new CoralConfigIONetworkTables(nt));

        SafeableSubsystem[] safeable = { m_elevator, m_algae, m_coral };
        m_climber = new ClimberSubsystem(new ClimberStingerIOSpark(),
                new ClimberPneumaticsIOReal(),
                safeable);
    }

    private void initSim() throws IllegalStateException {
        if (Constants.RobotMode.currentMode != Mode.SIM) {
            throw new IllegalStateException("initSim can only be called in SIM mode");
        }

        SimulatedArena.getInstance().resetFieldForAuto();

        final DriveTrainSimulationConfig simConf = DriveTrainSimulationConfig.Default()
                .withGyro(COTS.ofPigeon2())
                .withSwerveModule(new SwerveModuleSimulationConfig(
                        DCMotor.getNeoVortex(1), // drive motor
                        DCMotor.getNEO(1), // steer motor
                        1 / Constants.Modules.kDriveMotorGearRatio, // drive motor gear ratio
                        // TODO: whats the true value of this?
                        12.8, // steer motor gear ratio
                        Units.Volts.of(0.1), // drive friction, in voltage
                        Units.Volts.of(0.1), // steer friction, in voltage
                        Units.Meters.of(Constants.Modules.kWheelDiameterMeters / 2), // wheel
                                                                                     // radius
                        Units.KilogramSquareMeters.of(0.03), // steer rotational inertia
                        1.2 // wheel coefficient of friction
                )).withRobotMass(Units.Pounds.of(135))
                .withBumperSize(Units.Meters.of(Constants.Modules.kBumperLengthMeters),
                        Units.Meters.of(Constants.Modules.kBumperWidthMeters))
                .withTrackLengthTrackWidth(Units.Meters.of(Constants.Modules.kTrackLengthMeters),
                        Units.Meters.of(Constants.Modules.kTrackWidthMeters));

        Pose2d initialPose = new Pose2d(8, 4, new Rotation2d());

        final SwerveDriveSimulation dtSim = new SwerveDriveSimulation(simConf, initialPose);

        SimulatedArena.getInstance().addDriveTrainSimulation(dtSim);

        m_swerve = new Drivetrain(new GyroIOSim(dtSim.getGyroSimulation()),
                new SwerveModuleIOSim(dtSim.getModules()[0]),
                new SwerveModuleIOSim(dtSim.getModules()[1]),
                new SwerveModuleIOSim(dtSim.getModules()[2]),
                new SwerveModuleIOSim(dtSim.getModules()[3]));
        m_swerve.resetOdo(initialPose);

        driveSim = Optional.of(dtSim);

        m_vision = new Vision(m_swerve::addVisionMeasurement,
                Constants.ID.allAprilIDs,
                new VisionIOPhotonVisionSim("cam1",
                        new Transform3d(-Constants.Offsets.cameraOffsetForwardM, 0.0, 0.3,
                                Rotation3d.kZero),
                        dtSim::getSimulatedDriveTrainPose));

        ElevatorSim elevSim = new ElevatorSim(DCMotor.getNEO(2), 1 / Constants.Elevator.kElevatorGearRatio, 24.0,
                Elevator.kElevatorDrumRadius, 0.0, Constants.Elevator.kElevatorMaxPos, true, 0, 0.01, 0.0);
        // TODO: simulate elevator
        m_elevator = new ElevatorSubsystem(
                new ElevatorMotorIOSim(elevSim),
                new ElevatorLimitsIOSim(elevSim),
                new ElevatorConfigIONetworkTables(nt));

        // TODO: simulate Algae
        m_algae = new AlgaeGrabberSubsystem(new AlgaePneumaticsIO() {
        }, new AlgaeRetrievalIO() {
        });

        IntakeSimulation intakeSim = new IntakeSimulation("Coral", dtSim,
                new Triangle(new Vector2(0, 0), new Vector2(0.2, 0), new Vector2(0, 0.2)), 1);

        CoralGrabberIOSim grabberIo = new CoralGrabberIOSim(dtSim, intakeSim);
        m_coral = new CoralSubsystem(
                m_elevator,
                grabberIo,
                new CoralWristIOSim(),
                new CoralDetectionIOSim(intakeSim, dtSim::getSimulatedDriveTrainPose),
                new CoralConfigIONetworkTables(nt));

        intakeSim.addGamePieceToIntake();

        // this doesn't need to be simulated; it has little-to-no importance/ability to
        // tune in sim
        SafeableSubsystem[] safeable = { m_elevator, m_algae, m_coral };
        m_climber = new ClimberSubsystem(
                new ClimberStingerIO() {
                },
                new ClimberPneumaticsIO() {
                },
                safeable);
    }

    private void initReplay() throws IllegalStateException {
        if (Constants.RobotMode.currentMode != Mode.REPLAY) {
            throw new IllegalStateException("initReplay can only be called in REPLAY mode");
        }

        m_swerve = new Drivetrain(
                new GyroIO() {
                },
                new SwerveModuleIO() {
                },
                new SwerveModuleIO() {
                },
                new SwerveModuleIO() {
                },
                new SwerveModuleIO() {
                });

        m_vision = new Vision(m_swerve::addVisionMeasurement, Constants.ID.allAprilIDs, new VisionIO() {
        });

        m_elevator = new ElevatorSubsystem(
                new ElevatorMotorIO() {
                }, new ElevatorLimitsIO() {
                }, new ElevatorConfigIO() {
                });

        m_algae = new AlgaeGrabberSubsystem(new AlgaePneumaticsIO() {
        }, new AlgaeRetrievalIO() {
        });

        m_coral = new CoralSubsystem(
                m_elevator,
                new CoralGrabberIO() {
                },
                new CoralWristIO() {
                },
                new CoralDetectionIO() {
                },
                new CoralConfigIO() {
                });

        // this doesn't need to be simulated; it has little-to-no importance/ability to
        // tune in sim
        SafeableSubsystem[] safeable = { m_elevator, m_algae, m_coral };
        m_climber = new ClimberSubsystem(
                new ClimberStingerIO() {
                },
                new ClimberPneumaticsIO() {
                },
                safeable);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    public void configureBindings() {
        if (bindingsConfigured) {
            return;
        } else {
            bindingsConfigured = true;
        }

        // DRIVE CONTROLLERS BINDINGS

        // Bumper Buttons for Scoring Sequence
        Controller.kDriveController.rightBumper().and(m_climber::isCoralMode).onTrue(m_scoreRight);

        // Bumper Buttons for Scoring Sequence
        Controller.kDriveController.leftBumper().and(m_climber::isCoralMode).onTrue(m_scoreLeft);

        // Toggle Robot Oriented Drive
        Controller.kDriveController.back()
                .toggleOnTrue(this.m_swerve.toggleFieldRelativeCommand());

        // Cancel Coral Score
        Controller.kDriveController.a().onTrue(m_scoreCancel);

        // reset Field Orient Command
        Controller.kDriveController.y().onTrue((new ResetOdoCommand(m_swerve)));

        // Coral extend and retract
        Controller.kDriveController.x().and(m_climber::isCoralMode).onTrue(m_coral.wristExtendCommand());
        Controller.kDriveController.b().and(m_climber::isCoralMode).onTrue(m_coral.wristRetractCommand());

        // Trigger Buttons for Algae Intake and Eject
        Controller.kDriveController.leftTrigger().and(m_climber::isCoralMode)
                .whileTrue(new EjectAlgae(m_algae));
        Controller.kDriveController.rightTrigger().and(m_climber::isCoralMode)
                .whileTrue(new AlgaeIntake(m_algae)); // when disabling robot make
        // sure grabber isnt extended

        // MANIPULATOR CONTROLLER BINDINGS:

        // D-Pad for Elevator Manual Control
        Controller.kManipulatorController.povUp().and(m_climber::isCoralMode)
                .onTrue(m_elevator.runOnce(() -> m_elevator.manualUp()))
                .onFalse(m_elevator.runOnce(() -> m_elevator.stopManualMode()));
        Controller.kManipulatorController.povDown().and(m_climber::isCoralMode)
                .onTrue(m_elevator.runOnce(() -> m_elevator.manualDown()))
                .onFalse(m_elevator.runOnce(() -> m_elevator.stopManualMode()));

        // D-Pad for Stinger Control
        Controller.kManipulatorController.povLeft().whileTrue(new MoveStinger(m_climber, true));
        Controller.kManipulatorController.povRight().whileTrue(new MoveStinger(m_climber, false));

        // Bumper Buttons
        Controller.kManipulatorController.leftBumper()
                .onTrue(m_climber.runOnce(() -> m_climber.toggleGrabArms()));

        // Triger Buttons
        Controller.kManipulatorController.rightTrigger().onTrue(new EjectCoral(m_coral));
        Controller.kManipulatorController.rightBumper().and(m_climber::isCoralMode)
                .onTrue(new ConditionalCommand(buildRemoveAlgaeCommand(),
                        new RumbleController(Controller.kManipulatorController.getHID(), .5),
                        () -> (m_elevator.isValidAlgaeLevel() && Constants.ID.reefAprilIDs
                                .contains(m_vision.getFiducialID(0)))));

        // Back Button and Start button for Climber Mode Toggle
        Controller.kManipulatorController.back()
                .onTrue(m_climber.runOnce(() -> m_climber.setClimbMode()));
        Controller.kManipulatorController.start()
                .onTrue(m_climber.runOnce(() -> m_climber.setCoralMode()));

        // X, A, B, Y for Elevator Level Flags
        Controller.kManipulatorController.x()
                .onTrue(m_elevator.runOnce(() -> m_elevator.setLevelFlag(ElevatorLevel.LEVEL1)));
        Controller.kManipulatorController.a()
                .onTrue(m_elevator.runOnce(() -> m_elevator.setLevelFlag(ElevatorLevel.LEVEL2)));
        Controller.kManipulatorController.b()
                .onTrue(m_elevator.runOnce(() -> m_elevator.setLevelFlag(ElevatorLevel.LEVEL3)));
        Controller.kManipulatorController.y()
                .onTrue(m_elevator.runOnce(() -> m_elevator.setLevelFlag(ElevatorLevel.LEVEL4)));

        // TEMPORARY BINDINGS FOR TESTING:
        Controller.kDriveController.povUp().and(m_climber::isCoralMode)
                .whileTrue(new MoveCoralManipulator(m_coral, true));
        Controller.kDriveController.povDown().and(m_climber::isCoralMode)
                .whileTrue(new MoveCoralManipulator(m_coral, false));
        Controller.kDriveController.povLeft().and(m_climber::isCoralMode)
                .onTrue(m_elevator.runOnce(() -> m_elevator.levelDown()));
        Controller.kDriveController.povRight().and(m_climber::isCoralMode)
                .onTrue(m_elevator.runOnce(() -> m_elevator.levelUp()));
    }

    public void configureTestBindings() {
        if (bindingsConfigured) {
            return;
        } else {
            bindingsConfigured = true;
        }

        Controller.kDriveController.y().onTrue((new ResetOdoCommand(m_swerve)));

        Controller.kDriveController.back()
                .toggleOnTrue(this.m_swerve.toggleFieldRelativeCommand());

        // Controller.kManipulatorController.rightTrigger().whileTrue(new
        // ElevatorJoystick(Controller.kManipulatorController.getHID(), m_elevator));

        Controller.kDriveController.leftBumper().whileTrue(new RotateWheels(m_swerve, 0.0));
        Controller.kDriveController.rightBumper().whileTrue(new RotateWheels(m_swerve, 90.0));

        Controller.kDriveController.a().onTrue(new DriveOffset(m_swerve, m_vision, false));
        Controller.kDriveController.b().onTrue(new DriveOffset(m_swerve, m_vision, true));
        /*
         * Controller.kDriveController.x().onTrue(new DriveDistance(m_swerve,
         * () -> m_Limelight.getzDistanceMeters() - 0.1, 0));
         */

        // Test commands for centering on tag
        Controller.kDriveController.x().onTrue(new DriveDistance(m_swerve,
                () -> (m_vision.getTargetDistZ(0).in(Units.Meters) - Constants.Offsets.cameraOffsetFromFrontBumber)
                        + 0.02,
                0));

        Controller.kDriveController.leftTrigger().and(m_climber::isCoralMode)
                .whileTrue(new EjectAlgae(m_algae));
        Controller.kDriveController.rightTrigger().and(m_climber::isCoralMode)
                .whileTrue(new AlgaeIntake(m_algae));

        // when disabling robot, make
        // sure grabber isnt extended
        // Controller.kDriveController.leftBumper().onTrue(new DriveRange(m_swerve, ()
        // -> 0.5, () -> m_range.getRange(), 90, 0.2));

        Controller.kManipulatorController.povUp().and(m_climber::isCoralMode)
                .onTrue(m_elevator.runOnce(() -> m_elevator.levelUp()));
        Controller.kManipulatorController.povDown().and(m_climber::isCoralMode)
                .onTrue(m_elevator.runOnce(() -> m_elevator.levelDown()));

        Controller.kManipulatorController.povLeft().whileTrue(new MoveStinger(m_climber, true));
        Controller.kManipulatorController.povRight().whileTrue(new MoveStinger(m_climber, false));

        // TODO: make speed a constant
        Controller.kDriveController.povUp()
                .whileTrue(new DriveFixedVelocity(m_swerve, 0, () -> 0.5));
        Controller.kDriveController.povRight()
                .whileTrue(new DriveFixedVelocity(m_swerve, 90, () -> 0.5));
        Controller.kDriveController.povDown()
                .whileTrue(new DriveFixedVelocity(m_swerve, 180, () -> 0.5));
        Controller.kDriveController.povLeft()
                .whileTrue(new DriveFixedVelocity(m_swerve, 270, () -> 0.5));
        Controller.kDriveController.x().whileTrue(new EjectCoral(m_coral));

        Controller.kManipulatorController.leftBumper()
                .onTrue(m_climber.runOnce(() -> m_climber.toggleGrabArms()));
        Controller.kManipulatorController.back()
                .onTrue(m_climber.runOnce(() -> m_climber.setClimbMode()));
        Controller.kManipulatorController.start()
                .onTrue(m_climber.runOnce(() -> m_climber.setCoralMode()));

        Controller.kManipulatorController.x()
                .onTrue(m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.LEVEL1)));
        Controller.kManipulatorController.a()
                .onTrue(m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.LEVEL2)));
        Controller.kManipulatorController.b()
                .onTrue(m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.LEVEL3)));
        Controller.kManipulatorController.y()
                .onTrue(m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.LEVEL4)));
    }

    protected SequentialCommandGroup buildScoreOffsetCommand(boolean isLeft) {
        return new SequentialCommandGroup(
                new PrintCommand("Running offset score routine"),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new DriveOffset(m_swerve, m_vision, isLeft),
                                new StopDrive(m_swerve),
                                new StationaryWait(m_swerve, 0.06),
                                new DriveDistance(m_swerve, () -> 0.3,
                                        0).withTimeout(0.6),
                                new DriveDistance2(m_swerve,
                                        () -> (m_vision.getTargetDistZ(0).in(Units.Meters)
                                                - .42),
                                        0)
                                        .withTimeout(1),
                                new StopDrive(m_swerve)),
                        new GoToFlagLevel(m_elevator)),
                new PrintCommand("Running offset score routinex2"),
                new EjectCoral(m_coral),
                new StationaryWait(m_swerve, .5),
                new DriveDistance2(m_swerve, () -> 0.1, 180).withTimeout(.4),
                new StopDrive(m_swerve),
                m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.GROUND)));
    }

    protected SequentialCommandGroup buildScoreOffsetAutoCommand(boolean isLeft) {
        return new SequentialCommandGroup(
                new PrintCommand("Running offset score routine"),
                new DriveOffset(m_swerve, m_vision, isLeft),
                new ParallelCommandGroup(
                        new GoToFlagLevel(m_elevator),
                        new SequentialCommandGroup(
                                new DriveDistance2(m_swerve,
                                        () -> (m_vision.getTargetDistZ(0).in(Units.Meters)
                                                - .42),
                                        0)
                                        .withTimeout(1),
                                new PrintCommand("after DD?"),
                                new StopDrive(m_swerve))),
                new PrintCommand("---- x2"),
                new EjectCoral(m_coral),
                new StationaryWait(m_swerve, .4),
                new DriveDistance2(m_swerve, () -> 0.1, 180).withTimeout(.4),
                new StopDrive(m_swerve),
                m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.GROUND)));
    }

    protected SequentialCommandGroup buildScoreBumperedUpCommand(boolean isLeft, double forwardTimeout) {
        return new SequentialCommandGroup(
                new PrintCommand("Running drive left right score"),
                new ParallelCommandGroup(
                        // Raise the elevator to the selected level while in parallel aligning
                        // left or right
                        new GoToFlagLevel(m_elevator).withTimeout(2.5),
                        new SequentialCommandGroup(
                                new DriveDistance2(m_swerve, () -> 0.15, 0)
                                        .withTimeout(forwardTimeout),
                                // new DriveFixedVelocity(m_swerve, 0, () ->
                                // 0.5).withTimeout(0.2),
                                new DriveFixedVelocity(m_swerve, 180, () -> 0.25)
                                        .withTimeout(.1),
                                new DriveLeftOrRight(m_swerve, m_vision, isLeft),
                                new StopDrive(m_swerve))),
                new StopDrive(m_swerve),
                new EjectCoral(m_coral),
                new StationaryWait(m_swerve, .5),
                new DriveDistance2(m_swerve, () -> 0.1, 180).withTimeout(.4),
                new StopDrive(m_swerve),
                m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.GROUND)));
    }

    protected SequentialCommandGroup buildScoreBumperedUpAutoCommand(boolean isLeft, double forwardTimeout) {
        return new SequentialCommandGroup(
                new PrintCommand("Running drive left right score"),
                new ParallelCommandGroup(
                        // Raise the elevator to the selected level while in parallel aligning
                        // left or right
                        new GoToFlagLevel(m_elevator).withTimeout(2.5),
                        new SequentialCommandGroup(
                                // new DriveDistance(m_swerve, () -> 0.7,
                                // 0).withTimeout(forwardTimeout),
                                new DriveFixedVelocity(m_swerve, 0, () -> 1.5)
                                        .withTimeout(0.5),
                                new DriveFixedVelocity(m_swerve, 180, () -> 0.25)
                                        .withTimeout(.1),
                                new DriveLeftOrRight(m_swerve, m_vision, isLeft),
                                new StopDrive(m_swerve))),
                new StopDrive(m_swerve),
                new EjectCoral(m_coral),
                new StationaryWait(m_swerve, .5),
                new DriveDistance2(m_swerve, () -> 0.1, 180).withTimeout(.4),
                new StopDrive(m_swerve),
                m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.GROUND)));
    }

    public SequentialCommandGroup buildRemoveAlgaeCommand() {
        return new SequentialCommandGroup(
                new PrintCommand("Running remove algae"),
                new ParallelCommandGroup(
                        m_coral.wristExtendCommand(),
                        new DriveOffset(m_swerve, m_vision, 1.1, 0.0),
                        new GoToFlagLevel(m_elevator)),
                new DriveFixedVelocity(m_swerve, 0, () -> 2.25).withTimeout(0.8),
                new StopDrive(m_swerve),
                new PrintCommand("Remove algae complete"));
    }

    public Drivetrain getDrivetrain() {
        return this.m_swerve;
    }

    private void configurePathPlanner() {
        // autoChooser.addOption("DriveForward", "DriveForward"); // Permanent choice
        autoChooser.addOption("OnePieceAuto", "OnePieceAuto"); // Permanent choice
        autoChooser.addOption("Left2Piece", "Left2Piece"); // Permanent choice
        autoChooser.addOption("Right2Piece", "Right2Piece"); // Permanent choice
        // For multi-step, create name to be name of multi-step, then have object be the
        // name of the first step
        // MultiStep example below
        // autoChooser.addOption("MultiStepRight", "Starting2Reef2"); // Permanent
        // choice
        // autoChooser.addOption("MultiStepLeft", "Starting7Reef4"); // Permanent choice
    }

    public SequentialCommandGroup buildTwoPieceAuto(String pathToReef, int tag1,
            String pathToCoralStn, String pathCoralToReef, int tag2, double forwardDistM) {
        return new SequentialCommandGroup(
                m_swerve.runOnce(() -> m_swerve.setEnableVisionPoseInputs(false)),
                new StopDrive(m_swerve),
                getAutonomousCommand(pathToReef, true),
                m_elevator.runOnce(() -> m_elevator.setLevelFlag(ElevatorLevel.LEVEL4)),
                buildScoreOffsetAutoCommand(true),
                new StationaryWait(m_swerve, .1),
                new PrintCommand("finished first coral; going to coral"),
                getAutonomousCommand(pathToCoralStn, false),
                new StopDrive(m_swerve),
                // new StationaryWait(m_swerve, .05),
                new DriveDistance2(m_swerve, () -> .55, 180).withTimeout(0.7),
                new StopDrive(m_swerve),
                new StationaryWait(m_swerve, .4),
                getAutonomousCommand(pathCoralToReef, false),
                new StopDrive(m_swerve),
                // new StationaryWait(m_swerve, .05),
                buildScoreOffsetCommand(false),
                m_swerve.runOnce(() -> m_swerve.setEnableVisionPoseInputs(false)));
    }

    public SequentialCommandGroup buildTwoPieceAutoBumpered(String pathToReef, int tag1,
            String pathToCoralStn, String pathCoralToReef, int tag2, double forwardDistM) {
        return new SequentialCommandGroup(
                m_swerve.runOnce(() -> m_swerve.setEnableVisionPoseInputs(false)),
                getAutonomousCommand(pathToReef, true),
                new StationaryWait(m_swerve, 0.5), // Testing purposes
                buildScoreBumperedUpCommand(false, .25),
                new StationaryWait(m_swerve, 0.5), // Testing purposes
                getAutonomousCommand(pathToCoralStn, false),
                new DriveDistance2(m_swerve, () -> .1, 180).withTimeout(0.3),
                new StopDrive(m_swerve),
                new StationaryWait(m_swerve, 1.0),
                getAutonomousCommand(pathCoralToReef, false),
                new StopDrive(m_swerve),
                new StationaryWait(m_swerve, .5), // Testing purposes
                buildScoreBumperedUpCommand(false, .25),
                new StationaryWait(m_swerve, .5), // Testing purposes
                m_swerve.runOnce(() -> m_swerve.setEnableVisionPoseInputs(false)));
    }

    public void startAutonomous() {
        String auto = autoChooser.get();
        SequentialCommandGroup start;
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (auto.equals("Left2Piece")) { // For testing
            int tag1 = 20;
            int tag2 = 19;
            if (ally.isPresent()) {
                if (ally.get() == Alliance.Red) {
                    tag1 = 11;
                    tag2 = 6;
                }
            }
            start = buildTwoPieceAuto("Starting2Reef2",
                    tag1, "Reef2Player1",
                    "Player1Reef1", tag2, 0.16);
            start.schedule();
        } else if (auto.equals("Right2Piece")) {
            int tag1 = 22;
            int tag2 = 17;
            if (ally.isPresent()) {
                if (ally.get() == Alliance.Red) {
                    tag1 = 9;
                    tag2 = 8;
                }
            }
            start = buildTwoPieceAuto("Starting6Reef4",
                    tag1, "Reef4Player2",
                    "Player2Reef5", tag2, 0.16);
            start.schedule();
        } else if (auto.equals("OnePieceAuto")) {
            start = new SequentialCommandGroup(
                    m_swerve.runOnce(() -> m_swerve.setEnableVisionPoseInputs(false)),
                    m_elevator.runOnce(() -> m_elevator.setLevelFlag(ElevatorLevel.LEVEL4)),
                    getAutonomousCommand("OnePieceAuto", true),
                    new StationaryWait(m_swerve, .2),
                    // buildScoreBumperedUpAutoCommand(false, 1.5),
                    buildScoreOffsetCommand(false),
                    m_swerve.runOnce(() -> m_swerve.setEnableVisionPoseInputs(false)),
                    new DriveDistance2(m_swerve, () -> 0.5, 180),
                    m_elevator.runOnce(() -> m_elevator.setLevelFlag(ElevatorLevel.LEVEL1)),
                    buildRemoveAlgaeCommand(),
                    new DriveFixedVelocity(m_swerve, 180, () -> 2.25).withTimeout(0.4),
                    new StopDrive(m_swerve));
            start.schedule();
        } else {
            System.err.println("Invalid auto routine specified");
        }
    }

    public Command getAutonomousCommand(String pathName, boolean resetOdometry) {
        List<Waypoint> waypoints;
        PathPlannerPath path;
        Waypoint first;
        System.out.println("getAutoCommand building auto for " + pathName);
        final Optional<Translation2d> newTranslation;
        try {
            path = PathPlannerPath.fromPathFile(pathName);
            if (resetOdometry) {
                Optional<Pose2d> pose = path.getStartingHolonomicPose();
                Optional<Alliance> ally = DriverStation.getAlliance();
                waypoints = path.getWaypoints();
                first = waypoints.get(0);
                if (ally.orElse(Alliance.Blue) == Alliance.Red) {
                    System.out.println("Flipping start location for red");
                    first = first.flip();
                }
                if (pose.isPresent()) {
                    newTranslation = Optional.of(first.anchor());

                    System.out.println(first);
                } else {
                    newTranslation = Optional.empty();

                    System.out.println("Error getting PathPlanner pose");
                }
            } else {
                newTranslation = Optional.empty();
            }

            return new SequentialCommandGroup(
                    m_swerve.runOnce(() -> {
                        if (resetOdometry) {
                            Rotation2d newRot = m_swerve.resetGyro();

                            newTranslation.ifPresent((trans) -> {
                                m_swerve.resetStartingTranslation(trans);

                                driveSim.ifPresent((sim) -> sim.setSimulationWorldPose(new Pose2d(trans, newRot)));
                            });
                        }
                    }),
                    AutoBuilder.followPath(path));
        } catch (FileVersionException | IOException | ParseException e) {
            System.err.println("Error loading PathPlanner path");
            e.printStackTrace();
        }
        return new StopDrive(m_swerve);
    }

    public void setTeleDefaultCommand() {
        if (this.m_swerve.getDefaultCommand() == null) {
            this.m_swerve.setDefaultCommand(this.m_driveCommand);
        }
    }

    public void setAutoDefaultCommand() {
        if (this.m_swerve.getDefaultCommand() == null) {
            this.m_swerve.setDefaultCommand(this.m_driveCommand);
        }
    }

    public void clearDefaultCommand() {
        this.m_swerve.removeDefaultCommand();
    }

    public void reinitialize() {
        this.m_elevator.updateConstants();
        this.m_elevator.resetEncoder();
        this.m_coral.reinit();
        // this.setPIDConstants();
    }

    public void turnRumbleOff() {
        Controller.kManipulatorController.setRumble(RumbleType.kLeftRumble, 0.0);
        Controller.kManipulatorController.setRumble(RumbleType.kRightRumble, 0.0);
    }

    // TODO: set these in the drivetrain using an input
    // public void setPIDConstants() {
    // // Configure the drive train tuning constants from the dashboard
    // for (SwerveModule m : m_swerve.getSwerveModules()) {
    // m.getDrivePidController().setP(m_driveP.getDouble(Constants.DrivetrainConstants.drivePID[0]));
    // m.getDriveFeedForward().setKs(
    // m_driveFFStatic.getDouble(Constants.DrivetrainConstants.driveFeedForward[0]));
    // m.getDriveFeedForward().setKv(
    // m_driveFFVel.getDouble(Constants.DrivetrainConstants.driveFeedForward[1]));
    // m.getDriveFeedForward().setKa(m_driveAccel.getDouble(0.0));
    // m.getTurnPidController().setP(m_turnP.getDouble(Constants.DrivetrainConstants.turnPID[0]));
    // m.getTurnPidController().setI(m_turnI.getDouble(DrivetrainConstants.turnPID[1]));
    // m.getTurnFeedForward().setKs(m_turnFFStatic.getDouble(DrivetrainConstants.turnFeedForward[0]));
    // m.getTurnFeedForward().setKv(m_turnFFVel.getDouble(DrivetrainConstants.turnFeedForward[1]));
    // }
    // }

    public void simulationPeriodic() {

        if (Constants.RobotMode.currentMode == Constants.RobotMode.Mode.SIM) {
            SimulatedArena.getInstance().simulationPeriodic();
            Logger.recordOutput(
                    "FieldSimulation/Coral",
                    SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
            Logger.recordOutput(
                    "FieldSimulation/Algae",
                    SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));

            driveSim.ifPresent(dtSim -> Logger.recordOutput("FieldSimulation/SimPose",
                    dtSim.getSimulatedDriveTrainPose()));
        }
    }

    public void periodic() {
        // this _should_ be the only usage of SmartDashboard
        SmartDashboard.putData("Commands", CommandScheduler.getInstance());
    }
}
