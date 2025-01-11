// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Controller;
import frc.robot.Constants.ID;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Commands.Drive;
import frc.robot.Commands.ResetOdoCommand;

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
    private final SendableChooser<String> autoChooser;

    private ShuffleboardTab m_competitionTab = Shuffleboard.getTab("Competition Tab");
    private GenericEntry m_xVelEntry = m_competitionTab.add("Chassis X Vel", 0).getEntry();
    private GenericEntry m_yVelEntry = m_competitionTab.add("Chassis Y Vel", 0).getEntry();
    private GenericEntry m_gyroAngle = m_competitionTab.add("Gyro Angle", 0).getEntry();
    private SwerveModuleSB[] mSwerveModuleTelem;
    
    Command m_driveCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        this.m_gyro.getConfigurator().apply(new MountPoseConfigs().withMountPoseYaw(-90));
        this.m_swerve = new Drivetrain(m_gyro);
        
        SwerveModuleSB[] swerveModuleTelem = { new SwerveModuleSB("FR", m_swerve.getFrontRightSwerveModule(), m_competitionTab),
                              new SwerveModuleSB("FL", m_swerve.getFrontLeftSwerveModule(), m_competitionTab),
                              new SwerveModuleSB("BR", m_swerve.getBackRightSwerveModule(), m_competitionTab),
                              new SwerveModuleSB("BL", m_swerve.getBackLeftSwerveModule(), m_competitionTab)};
        mSwerveModuleTelem = swerveModuleTelem;

        // Xbox controllers return negative values when we push forward.
        this.m_driveCommand = new Drive(m_swerve);
        this.m_swerve.setDefaultCommand(this.m_driveCommand);

        autoChooser = new SendableChooser<>(); // Default auto will be `Commands.none()'

        configurePathPlanner();
        autoChooser.setDefaultOption("DO NOTHING!", "NO AUTO");
        m_competitionTab.add("Auto Chooser", autoChooser).withSize(2, 1).withPosition(7, 0);
        m_competitionTab.add("Drivetrain", this.m_swerve);
        configureBindings();
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
    private void configureBindings() {
        Controller.kDriveController.y().onTrue((new ResetOdoCommand(m_swerve)));
        Controller.kDriveController.rightBumper()
                .onTrue(this.m_swerve.setFieldRelativeCommand(false))
                .onFalse(this.m_swerve.setFieldRelativeCommand(true));

        Controller.kDriveController.leftBumper().onTrue(m_swerve.setDriveMultCommand(0.5))
                .onFalse(m_swerve.setDriveMultCommand(1));
    }

    public Drivetrain getDrivetrain() {
        return this.m_swerve;
    }

    private void configurePathPlanner() {
        autoChooser.addOption("Vision Test",   "Vision Test");
        autoChooser.addOption("SwerveTestAuto25", "SwerveTestAuto25");
    }

    public Command getAutonomousCommand() {
        if (autoChooser.getSelected().equals("NO AUTO")) {
            return Commands.none();
        }
        System.out.println("getAutoCommand building auto for " + autoChooser.getSelected());
        return AutoBuilder.buildAuto(autoChooser.getSelected());
    }

    public void setTeleDefaultCommand() {
        if (this.m_swerve.getDefaultCommand() == null) {
            this.m_swerve.setDefaultCommand(this.m_driveCommand);
        }
    }

    public void setAutoDefaultCommand() {
    }

    public void clearDefaultCommand() {
        this.m_swerve.removeDefaultCommand();
    }

    public void reportTelemetry() {
        m_xVelEntry.setDouble(m_swerve.getChassisSpeeds().vxMetersPerSecond);
        m_yVelEntry.setDouble(m_swerve.getChassisSpeeds().vyMetersPerSecond);
        m_gyroAngle.setDouble(m_swerve.getGyroYawRotation2d().getDegrees());
        for (SwerveModuleSB sb : mSwerveModuleTelem) {
            sb.update();
        }
    }
}
