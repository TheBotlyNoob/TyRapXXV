// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Controller;

public class Robot extends TimedRobot { 
    private RobotContainer m_containter;
    private Command autonomousCommand;

    @Override
    public void robotInit() {
        m_containter = new RobotContainer();
        autonomousCommand = m_containter.getAutonomousCommand();
    }

    @Override
    public void autonomousInit() {
        autonomousCommand.schedule();
    }

    @Override
    public void teleopInit() {
        autonomousCommand.cancel();
    }

    @Override
    public void autonomousPeriodic() {
        if(autonomousCommand.isFinished()) {
            System.out.println("FINISHED AUTO!");
        }
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
    }

    
}
