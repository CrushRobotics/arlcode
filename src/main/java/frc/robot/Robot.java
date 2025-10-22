// Copyright (c) FIRS T and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera; // Import HttpCamera
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
    // We will explicitly create HttpCamera objects and add them to the CameraServer.
    // This is an alternative to CameraServer.addHttpCamera() and can resolve
    // some dependency or class path issues.

    // Create and add the left camera
    HttpCamera limelightLeft = new HttpCamera("limelight-left", "http://limelight-left.local:5801/stream.mjpg");
    CameraServer.addCamera(limelightLeft);

    // Create and add the right camera
    HttpCamera limelightRight = new HttpCamera("limelight-right", "http://limelight-right.local:5801/stream.mjpg");
    CameraServer.addCamera(limelightRight);
  
    m_robotContainer = new RobotContainer();

    // Used to track usage of the KitBot code, please do not remove
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

  /**
   * teleopPeriodic is intentionally left empty.
   * All teleoperated control is handled by the CommandScheduler through
   * the default drive command set in RobotContainer. This prevents teleop
   * code from interfering with autonomous commands.
   */
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
    // Any specific initialization for simulation can go here.
    m_robotContainer.simulationInit();
  }

  @Override
  public void simulationPeriodic() {
    // This method is called approximately every 20ms in simulation.
    m_robotContainer.simulationPeriodic();
  }
}

