// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot {
  
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    HttpCamera limelightRight = new HttpCamera("limelight-right", "http://limelight-right.local:5800/stream.mjpg");
    CameraServer.startAutomaticCapture(limelightRight);

    HttpCamera limelightLeft = new HttpCamera("limelight-left", "http://limelight-left.local:5801/stream.mjpg");
    CameraServer.startAutomaticCapture(limelightLeft);
  
    m_robotContainer = new RobotContainer();

    HAL.report(tResourceType.kResourceType_Framework, 9);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }
  
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
    m_robotContainer.simulationInit();
  }

  @Override
  public void simulationPeriodic() {
    m_robotContainer.simulationPeriodic();
  }
  
}

