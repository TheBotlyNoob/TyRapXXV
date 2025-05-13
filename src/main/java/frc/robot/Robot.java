// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.StopDrive;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

public class Robot extends LoggedRobot {
  private RobotContainer m_container;
  protected boolean gyroCorrect;
  protected Alliance currentAlliance;

  @Override
  public void robotInit() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.RobotMode.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Initialize URCL
    Logger.registerURCL(URCL.startExternal());

    // Start AdvantageKit logger
    Logger.start();

    gyroCorrect = false;
    m_container = new RobotContainer();
    currentAlliance = m_container.getDrivetrain().getAlliance();

    FollowPathCommand.warmupCommand().schedule();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  @Override
  public void autonomousInit() {
    m_container.getDrivetrain().setFieldRelative(true);
    m_container.clearDefaultCommand();
    m_container.setAutoDefaultCommand();
    m_container.reinitialize();
    m_container.startAutonomous();
  }

  @Override
  public void teleopInit() {
    StopDrive stop = new StopDrive(m_container.getDrivetrain());
    stop.schedule();
    m_container.getDrivetrain().setFieldRelative(true);
    m_container.clearDefaultCommand();
    m_container.setTeleDefaultCommand();
    m_container.reinitialize();
    m_container.configureBindings();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_container.reportTelemetry();
  }

  @Override
  public void disabledPeriodic() {
    if (currentAlliance != m_container.getDrivetrain().getAlliance()) {
      currentAlliance = m_container.getDrivetrain().getAlliance();
      System.out.println("Alliance changed to " + currentAlliance);
      gyroCorrect = false;
    }
    if (!gyroCorrect) {
      double expectedGyro = m_container.getDrivetrain().getExpectedStartGyro();
      double currentGyro = m_container.getDrivetrain().getGyroYawRotation2d().getDegrees();
      System.out.println("Expected Gyro: " + expectedGyro + " current: " + currentGyro);
      if (Math.abs(expectedGyro - currentGyro) > 1.0) {
        System.out.println("Gyro delta too large");
        m_container.getDrivetrain().resetGyro();
        System.out.println("Reset gyro from robot periodic");
      } else {
        gyroCorrect = true;
        m_container.getDrivetrain().resetOdo();
      }
    }
    // Uncomment these lines in order to output the swerve turn encoder values (to obtain offsets)
    // SmartDashboard.putNumber("BackLeft",
    // m_container.getDrivetrain().getBackLeftSwerveModule().getRawTurningPositionRadians());
    // SmartDashboard.putNumber("BackRight",
    // m_container.getDrivetrain().getBackRightSwerveModule().getRawTurningPositionRadians());
    // SmartDashboard.putNumber("FrontLeft",
    // m_container.getDrivetrain().getFrontLeftSwerveModule().getRawTurningPositionRadians());
    // SmartDashboard.putNumber("FrontRight",
    // m_container.getDrivetrain().getFrontRightSwerveModule().getRawTurningPositionRadians());
  }

  @Override
  public void testInit() {
    // m_container.getDrivetrain().setFieldRelative(true);
    // m_container.clearDefaultCommand();
    // m_container.setTeleDefaultCommand();
    m_container.reinitialize();
    m_container.configureTestBindings();
  }

  @Override
  public void testPeriodic() {}
}
