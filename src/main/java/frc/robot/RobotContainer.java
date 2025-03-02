// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.Optional;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.Subsystems.AlgaeGrabberSubsystem;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem.ElevatorLevel;
import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.RangeSensor;
import frc.robot.Subsystems.CoralSubsystem;
import frc.robot.Commands.AlgaeIntake;
import frc.robot.Commands.CenterOnTag;
import frc.robot.Commands.Drive;
import frc.robot.Commands.DriveDistance;
import frc.robot.Commands.DriveLeftOrRight;
import frc.robot.Commands.DriveFixedVelocity;
import frc.robot.Commands.DriveOffset;
import frc.robot.Commands.EjectAlgae;
import frc.robot.Commands.EjectCoral;
import frc.robot.Commands.GoToFlagLevel;
import frc.robot.Commands.GoToLevel;
import frc.robot.Commands.MoveCoralManipulator;
import frc.robot.Commands.MoveStinger;
import frc.robot.Commands.ResetOdoCommand;
import frc.robot.Commands.StationaryWait;
import frc.robot.Commands.StopDrive;
import org.json.simple.parser.ParseException;

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
    private final Pigeon2 m_gyro = new Pigeon2(ID.kGyro);
    private final Drivetrain m_swerve;
    private final Limelight m_Limelight;
    // remember to set this to final, commented out range code bc robot doesnt have
    // canrange yet
    private RangeSensor m_range;
    private final AlgaeGrabberSubsystem m_algae;
    private final ClimberSubsystem m_climber;
    private final SendableChooser<String> autoChooser;
    protected final ElevatorSubsystem m_elevator;
    protected final CoralSubsystem m_coral;

    private ShuffleboardTab m_competitionTab = Shuffleboard.getTab("Competition Tab");
    private GenericEntry m_xVelEntry = m_competitionTab.add("Chassis X Vel", 0).getEntry();
    private GenericEntry m_yVelEntry = m_competitionTab.add("Chassis Y Vel", 0).getEntry();
    private GenericEntry m_gyroAngle = m_competitionTab.add("Gyro Angle", 0).getEntry();
    private GenericEntry m_currentRange = m_competitionTab.add("Range", 0).getEntry();
    private GenericEntry m_commandedXVel = m_competitionTab.add("CommandedVX", 0).getEntry();
    private GenericEntry m_commandedYVel = m_competitionTab.add("CommandedVY", 0).getEntry();
    private StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
    protected GenericEntry m_driveP = m_competitionTab.add("Drive P Val", DriveTrainConstants.drivePID[0]).getEntry();
    protected GenericEntry m_driveFFStatic = m_competitionTab.add("Drive FF Static", DriveTrainConstants.driveFeedForward[0]).getEntry();
    protected GenericEntry m_driveFFVel = m_competitionTab.add("Drive FF Vel", DriveTrainConstants.driveFeedForward[1]).getEntry();
    protected GenericEntry m_driveAccel = m_competitionTab.add("Drive FF Accel", 0.0).getEntry();
    private GenericEntry m_fixedSpeed = m_competitionTab.add("Fixed Speed", 0).getEntry();
    private SwerveModuleSB[] mSwerveModuleTelem;

    Command m_driveCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        this.m_gyro.getConfigurator().apply(new MountPoseConfigs().withMountPoseYaw(-90));
        this.m_swerve = new Drivetrain(m_gyro);

        SwerveModuleSB[] swerveModuleTelem = {
                new SwerveModuleSB("FR", m_swerve.getFrontRightSwerveModule(), m_competitionTab),
                new SwerveModuleSB("FL", m_swerve.getFrontLeftSwerveModule(), m_competitionTab),
                new SwerveModuleSB("BR", m_swerve.getBackRightSwerveModule(), m_competitionTab),
                new SwerveModuleSB("BL", m_swerve.getBackLeftSwerveModule(), m_competitionTab) };
        mSwerveModuleTelem = swerveModuleTelem;

        this.m_Limelight = new Limelight();
        this.m_Limelight.setLimelightPipeline(2);
        CameraServer.startAutomaticCapture();
        this.m_algae = new AlgaeGrabberSubsystem(NetworkTableInstance.getDefault());
        this.m_climber = new ClimberSubsystem(m_swerve.getBackLeftSwerveModule().getTurnMotor().getAbsoluteEncoder(),
                NetworkTableInstance.getDefault());

        // this.m_range = new RangeSensor(0);
        this.m_elevator = new ElevatorSubsystem(NetworkTableInstance.getDefault());
        this.m_coral = new CoralSubsystem(NetworkTableInstance.getDefault());

        // Xbox controllers return negative values when we push forward.
        this.m_driveCommand = new Drive(m_swerve);
        this.m_swerve.setDefaultCommand(this.m_driveCommand);

        autoChooser = new SendableChooser<>(); // Default auto will be `Commands.none()'

        configurePathPlanner();
        autoChooser.setDefaultOption("DO NOTHING!", "NO AUTO");
        m_competitionTab.add("Auto Chooser", autoChooser).withSize(2, 1).withPosition(7, 0);

        m_competitionTab.add("Drivetrain", this.m_swerve);

        configureBindings();
        NamedCommands.registerCommand("StopDrive", new StopDrive(m_swerve));
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

        // DRIVE CONTROLLERS BINDINGS 

            //Bumper Buttons for Scoring Sequence
            Controller.kDriveController.rightBumper().onTrue(new SequentialCommandGroup(
                new DriveOffset(m_swerve, m_Limelight, false),
                new StopDrive(m_swerve),
                new StationaryWait(m_swerve, 0.06),
                new DriveDistance(m_swerve, () -> 0.15,0),
                new StopDrive(m_swerve),
                new GoToFlagLevel(m_elevator),
                new EjectCoral(m_coral),
                new WaitCommand(1),
                m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.GROUND))
            ));
            Controller.kDriveController.leftBumper().onTrue(new SequentialCommandGroup(
                new DriveOffset(m_swerve, m_Limelight, true),
                new StopDrive(m_swerve),
                new StationaryWait(m_swerve, 0.06),
                new DriveDistance(m_swerve, () -> 0.15,0),
                new StopDrive(m_swerve),
                new GoToFlagLevel(m_elevator),
                new EjectCoral(m_coral),
                new WaitCommand(1),
                m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.GROUND))
            ));

            //Toggle  Robot Oriented Drive
            Controller.kDriveController.back()
                    .toggleOnTrue(this.m_swerve.toggleFieldRelativeCommand());

            //reset Field Orient Command 
            Controller.kDriveController.y().onTrue((new ResetOdoCommand(m_swerve)));

            //Trigger Buttons for Algae Intake and Eject
            Controller.kDriveController.leftTrigger().whileTrue(new EjectAlgae(m_algae));
            Controller.kDriveController.rightTrigger().whileTrue(new AlgaeIntake(m_algae)); // when disabling robot make
                                                                                            // sure grabber isnt extended

        //MANIPULATOR CONTROLLER BINDINGS:

            //D-Pad for Elevator Manual Control
            Controller.kManipulatorController.povUp()
                    .onTrue(m_elevator.runOnce(() -> m_elevator.manualUp()))
                    .onFalse(m_elevator.runOnce(() -> m_elevator.stopManualMode()));
            Controller.kManipulatorController.povDown()
                    .onTrue(m_elevator.runOnce(() -> m_elevator.manualDown()))
                    .onFalse(m_elevator.runOnce(() -> m_elevator.stopManualMode()));

            //D-Pad for Stinger Control
            Controller.kManipulatorController.povLeft().whileTrue(new MoveStinger(m_climber, true));
            Controller.kManipulatorController.povRight().whileTrue(new MoveStinger(m_climber, false));

            //Bumper Buttons 
            Controller.kManipulatorController.leftBumper()
                    .onTrue(m_climber.runOnce(() -> m_climber.toggleGrabArms()));

            //Triger Buttons 
            Controller.kManipulatorController.rightTrigger().whileTrue(new EjectCoral(m_coral));

            //Back Button for Climber Mode Toggle
            Controller.kManipulatorController.back()
                    .onTrue(m_climber.runOnce(() -> m_climber.toggleClimbMode()));
            
            //X, A, B, Y for Elevator Level Flags
            Controller.kManipulatorController.x()
                    .onTrue(m_elevator.runOnce(() -> m_elevator.setLevelFlag(ElevatorLevel.LEVEL1)));
            Controller.kManipulatorController.a()
                    .onTrue(m_elevator.runOnce(() -> m_elevator.setLevelFlag(ElevatorLevel.LEVEL2)));
            Controller.kManipulatorController.b()
                    .onTrue(m_elevator.runOnce(() -> m_elevator.setLevelFlag(ElevatorLevel.LEVEL3)));
            Controller.kManipulatorController.y()
                    .onTrue(m_elevator.runOnce(() -> m_elevator.setLevelFlag(ElevatorLevel.LEVEL4)));

        //TEMPORARY BINDINGS FOR TESTING:
            Controller.kDriveController.povUp().whileTrue(new MoveCoralManipulator(m_coral, true));
            Controller.kDriveController.povDown().whileTrue(new MoveCoralManipulator(m_coral, false));
            Controller.kDriveController.povLeft().onTrue(m_elevator.runOnce(() -> m_elevator.levelDown()));
            Controller.kDriveController.povRight().onTrue(m_elevator.runOnce(() -> m_elevator.levelUp()));
    }

    public void configureTestBindings() {
        Controller.kDriveController.y().onTrue((new ResetOdoCommand(m_swerve)));

        Controller.kDriveController.back()
                .toggleOnTrue(this.m_swerve.toggleFieldRelativeCommand());

        //Controller.kManipulatorController.rightTrigger().whileTrue(new ElevatorJoystick(m_elevator));

        Controller.kDriveController.leftBumper().onTrue(m_swerve.setDriveMultCommand(0.5))
                .onFalse(m_swerve.setDriveMultCommand(1));
        Controller.kDriveController.a().onTrue(new DriveOffset(m_swerve, m_Limelight, false));
        Controller.kDriveController.b().onTrue(new DriveOffset(m_swerve, m_Limelight, true));
        /*Controller.kDriveController.x().onTrue(new DriveDistance(m_swerve,
                () -> m_Limelight.getzDistanceMeters() - 0.1, 0));*/

        // Test commands for centering on tag
        Controller.kDriveController.x().onTrue(new DriveDistance(m_swerve,
                () -> (m_Limelight.getzDistanceMeters() - Constants.Offsets.cameraOffsetFromFrontBumber) + 0.02, 0));

        Controller.kDriveController.leftTrigger().whileTrue(new EjectAlgae(m_algae));
        Controller.kDriveController.rightTrigger().whileTrue(new AlgaeIntake(m_algae)); // when disabling robot make
                                                                                        // sure grabber isnt extended

        
        Controller.kManipulatorController.povUp()
                .onTrue(m_elevator.runOnce(() -> m_elevator.manualUp()))
                .onFalse(m_elevator.runOnce(() -> m_elevator.stopManualMode()));
        Controller.kManipulatorController.povDown()
                .onTrue(m_elevator.runOnce(() -> m_elevator.manualDown()))
                .onFalse(m_elevator.runOnce(() -> m_elevator.stopManualMode()));

        Controller.kManipulatorController.povLeft().whileTrue(new MoveStinger(m_climber, true));
        Controller.kManipulatorController.povRight().whileTrue(new MoveStinger(m_climber, false));


        Controller.kDriveController.povUp().whileTrue(new DriveFixedVelocity(m_swerve, 0, () -> m_fixedSpeed.getDouble(0.5)));
        Controller.kDriveController.povRight().whileTrue(new DriveFixedVelocity(m_swerve, 90, () -> m_fixedSpeed.getDouble(0.5)));
        Controller.kDriveController.povDown().whileTrue(new DriveFixedVelocity(m_swerve, 180, () -> m_fixedSpeed.getDouble(0.5)));
        Controller.kDriveController.povLeft().whileTrue(new DriveFixedVelocity(m_swerve, 270, () -> m_fixedSpeed.getDouble(0.5)));
        Controller.kDriveController.x().whileTrue(new EjectCoral(m_coral));
        
        Controller.kManipulatorController.leftBumper()
                .onTrue(m_climber.runOnce(() -> m_climber.toggleGrabArms()));
        Controller.kManipulatorController.back()
                .onTrue(m_climber.runOnce(() -> m_climber.toggleClimbMode()));

        Controller.kManipulatorController.x()
                .onTrue(m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.LEVEL1)));
        Controller.kManipulatorController.a()
                .onTrue(m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.LEVEL2)));
        Controller.kManipulatorController.b()
                .onTrue(m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.LEVEL3)));
        Controller.kManipulatorController.y()
                .onTrue(m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.LEVEL4)));
    }

    public Drivetrain getDrivetrain() {
        return this.m_swerve;
    }

    private void configurePathPlanner() {
        autoChooser.addOption("Starting2Reef2", "Starting2Reef2"); // Testing
        autoChooser.addOption("DriveForward", "DriveForward"); // Permanent choice
        autoChooser.addOption("OnePieceAuto", "OnePieceAuto"); // Permanent choice
        autoChooser.addOption("Player1Reef1", "Player1Reef1"); // Testing
        autoChooser.addOption("Reef2Player1", "Reef2Player1");
        autoChooser.addOption("Starting1Reef2", "Starting1Reef2");
        autoChooser.addOption("Starting7Reef4", "Starting7Reef4"); // Testing
        // For multi-step, create name to be name of multi-step, then have object be the name of the first step
        // MultiStep example below
        // autoChooser.addOption("MultiStepRight", "Starting2Reef2"); // Permanent choice
        // autoChooser.addOption("MultiStepLeft", "Starting7Reef4"); // Permanent choice
    }

    public SequentialCommandGroup buildStarting1Reef2Auto() {
        return new SequentialCommandGroup(
                    getAutonomousCommand(autoChooser.getSelected(), true), 
                    new StopDrive(m_swerve),
                    new StationaryWait(m_swerve, 0.5),
                    new DriveOffset(m_swerve, m_Limelight, false, 19),
                    new ParallelCommandGroup(new SequentialCommandGroup(
                    new DriveDistance(m_swerve, () -> 0.18, 0),
                    new StopDrive(m_swerve)),
                    new GoToLevel(m_elevator, ElevatorLevel.LEVEL2)),
                    new EjectCoral(m_coral),
                    new StationaryWait(m_swerve, .7),
                    m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.GROUND)),
                    new StationaryWait(m_swerve, 1.0),
                    getAutonomousCommand("Reef2Player1", false), 
                    new StopDrive(m_swerve),
                    new StationaryWait(m_swerve, .2),
                    new DriveDistance(m_swerve, () -> .02, 180),
                    new StopDrive(m_swerve),
                    new StationaryWait(m_swerve, 1.2),
                    getAutonomousCommand("Player1Reef1", false),
                    new StopDrive(m_swerve),
                    new StationaryWait(m_swerve, .4),
                    new DriveOffset(m_swerve, m_Limelight, true, 18),
                    new ParallelCommandGroup(new SequentialCommandGroup(
                    new DriveDistance(m_swerve, () -> 0.14, 0),
                    new StopDrive(m_swerve)),
                    new GoToLevel(m_elevator, ElevatorLevel.LEVEL2)),
                    new EjectCoral(m_coral),
                    new StationaryWait(m_swerve, .7),
                    m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.GROUND))
                    ); // Add Elevator to L4 and score piece
    }

    public SequentialCommandGroup buildStarting7Reef4Auto() {
        return new SequentialCommandGroup(
                    getAutonomousCommand(autoChooser.getSelected(), true), 
                    new StopDrive(m_swerve),
                    new StationaryWait(m_swerve, 0.5),
                    new DriveOffset(m_swerve, m_Limelight, false, 19), // Temporary id for testing
                    new ParallelCommandGroup(new SequentialCommandGroup(
                    new DriveDistance(m_swerve, () -> 0.18, 0),
                    new StopDrive(m_swerve)),
                    new GoToLevel(m_elevator, ElevatorLevel.LEVEL2)),
                    new EjectCoral(m_coral),
                    new StationaryWait(m_swerve, .7),
                    m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.GROUND)),
                    new StationaryWait(m_swerve, 1.0),
                    getAutonomousCommand("Reef4Player2", false), 
                    new StopDrive(m_swerve),
                    new StationaryWait(m_swerve, .2),
                    new DriveDistance(m_swerve, () -> .02, 180),
                    new StopDrive(m_swerve),
                    new StationaryWait(m_swerve, 1.2),
                    getAutonomousCommand("Player2Reef5", false),
                    new StopDrive(m_swerve),
                    new StationaryWait(m_swerve, .4),
                    new DriveOffset(m_swerve, m_Limelight, true, 18), // Temporary ID for testing
                    new ParallelCommandGroup(new SequentialCommandGroup(
                    new DriveDistance(m_swerve, () -> 0.14, 0),
                    new StopDrive(m_swerve)),
                    new GoToLevel(m_elevator, ElevatorLevel.LEVEL2)),
                    new EjectCoral(m_coral),
                    new StationaryWait(m_swerve, .7),
                    m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.GROUND))
                    ); 
    }

    public void startAutonomous() {
        m_coral.reinit();
        String auto = autoChooser.getSelected();
        /*SequentialCommandGroup start = new SequentialCommandGroup(getAutonomousCommand(autoChooser
                .getSelected())); */
        SequentialCommandGroup start;
        if (auto.equals("Starting1Reef2")) { // For testing
            start = buildStarting1Reef2Auto();
            start.schedule();
        } else if (auto.equals("Starting7Reef4")) {
            start = buildStarting7Reef4Auto();
            start.schedule();
        }
        
        /*else if(auto.equals("OnePieceAuto")){ // Permanent choice
            start = new SequentialCommandGroup(getAutonomousCommand(autoChooser
                    .getSelected()),
                new DriveOffset(m_swerve, m_Limelight, false)); // Add Elevator to L4 and score piece
        } /*else if (auto.equals("Starting2Reef2")) {
            // MultiStepRight example/template below
            start = new SequentialCommandGroup(getAutonomousCommand(autoChooser
                    .getSelected()),
                    new DriveOffset(m_swerve, m_Limelight, false, id?), // Add Elevator to L4 and score piece
                    getAutonomousCommand("Reef2Player1"), new CenterOnTag(m_swerve, m_Limelight), // Collect coral here
                    getAutonomousCommand("Player1Reef1"), new DriveOffset(m_swerve, m_Limelight, false, 19), // Add Elevator to L4 and score piece
                    getAutonomousCommand("Reef1Player1"), new CenterOnTag(m_swerve, m_Limelight), // Collect coral here
                    getAutonomousCommand("Player1Reef1"), new DriveOffset(m_swerve, m_Limelight, true, 19), // Add Elevator to L4 and score piece
                    getAutonomousCommand("Reef1Player1"), new CenterOnTag(m_swerve, m_Limelight), // Collect coral here
                    getAutonomousCommand("Player1Reef6"), new DriveOffset(m_swerve, m_Limelight, false, 18) // Add Elevator to L4 and score piece
                    );
        }*/
    }

    public Command getAutonomousCommand(String pathName, boolean resetOdometry) {
        PathPlannerPath path;
        System.out.println("getAutoCommand building auto for " + pathName);
        try {
            path = PathPlannerPath.fromPathFile(pathName);
            if (resetOdometry) {
                Optional<Pose2d> pose = path.getStartingHolonomicPose();
                if (pose.isPresent()) {
                    m_swerve.resetStartingPose(pose.get());
                    System.out.println(pose.get());
                } else {
                    System.out.println("Error getting PathPlanner pose");
                }
            }
            return AutoBuilder.followPath(path);
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

    public void updateConstants() {
        this.m_elevator.updateConstants();
        this.m_elevator.resetEncoder();
        this.m_coral.reinit();
        this.setPIDConstants();
    }

    public void setPIDConstants() {
        // Configure the drive train tuning constants from the dashboard
        double driveP = m_driveP.getDouble(Constants.DriveTrainConstants.drivePID[0]);
        double driveFFStatic = m_driveFFStatic.getDouble(Constants.DriveTrainConstants.driveFeedForward[0]);
        double driveFFVel = m_driveFFVel.getDouble(Constants.DriveTrainConstants.driveFeedForward[1]);
        double driveFFAccel = m_driveAccel.getDouble(0.0);
        m_swerve.getFrontLeftSwerveModule().getDrivePidController().setP(driveP);
        m_swerve.getFrontLeftSwerveModule().getDriveFeedForward().setKs(driveFFStatic);
        m_swerve.getFrontLeftSwerveModule().getDriveFeedForward().setKv(driveFFVel);
        m_swerve.getFrontLeftSwerveModule().getDriveFeedForward().setKa(driveFFAccel);
        m_swerve.getFrontRightSwerveModule().getDrivePidController().setP(driveP);
        m_swerve.getFrontRightSwerveModule().getDriveFeedForward().setKs(driveFFStatic);
        m_swerve.getFrontRightSwerveModule().getDriveFeedForward().setKv(driveFFVel);
        m_swerve.getFrontRightSwerveModule().getDriveFeedForward().setKa(driveFFAccel);
        m_swerve.getBackLeftSwerveModule().getDrivePidController().setP(driveP);
        m_swerve.getBackLeftSwerveModule().getDriveFeedForward().setKs(driveFFStatic);
        m_swerve.getBackLeftSwerveModule().getDriveFeedForward().setKv(driveFFVel);
        m_swerve.getBackLeftSwerveModule().getDriveFeedForward().setKa(driveFFAccel);
        m_swerve.getBackRightSwerveModule().getDrivePidController().setP(driveP);
        m_swerve.getBackRightSwerveModule().getDriveFeedForward().setKs(driveFFStatic);
        m_swerve.getBackRightSwerveModule().getDriveFeedForward().setKv(driveFFVel);
        m_swerve.getBackRightSwerveModule().getDriveFeedForward().setKa(driveFFAccel);
    }

    public void reportTelemetry() {
        m_xVelEntry.setDouble(m_swerve.getChassisSpeeds().vxMetersPerSecond);
        m_yVelEntry.setDouble(m_swerve.getChassisSpeeds().vyMetersPerSecond);
        m_gyroAngle.setDouble(m_swerve.getGyroYawRotation2d().getDegrees());
        for (SwerveModuleSB sb : mSwerveModuleTelem) {
            sb.update();
        }
        SwerveModuleState[] states = {
                m_swerve.getBackLeftSwerveModule().getState(),
                m_swerve.getBackRightSwerveModule().getState(),
                m_swerve.getFrontLeftSwerveModule().getState(),
                m_swerve.getFrontRightSwerveModule().getState() };
        publisher.set(states);
        // m_currentRange.setDouble(m_range.getRange());
        ChassisSpeeds commandedSpeeds = m_swerve.getCommandeChassisSpeeds();
        m_commandedXVel.setDouble(commandedSpeeds.vxMetersPerSecond);
        m_commandedYVel.setDouble(commandedSpeeds.vyMetersPerSecond);
    }
}
