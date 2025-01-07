// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private RobotContainer m_container;

    @Override
    public void robotInit() {
        m_container = new RobotContainer();
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void teleopInit() {
        m_container.getDrivetrain().setFieldRelative(true);
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledPeriodic() {
        // Uncomment these lines in order to output the swerve turn encoder values (to obtain offsets)
        //SmartDashboard.putNumber("BackLeft", m_container.getDrivetrain().getBackLeftSwerveModule().getRawTurningPositionRadians());
        //SmartDashboard.putNumber("BackRight", m_container.getDrivetrain().getBackRightSwerveModule().getRawTurningPositionRadians());
        //SmartDashboard.putNumber("FrontLeft", m_container.getDrivetrain().getFrontLeftSwerveModule().getRawTurningPositionRadians());
        //SmartDashboard.putNumber("FrontRight", m_container.getDrivetrain().getFrontRightSwerveModule().getRawTurningPositionRadians());
    }

}
