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


// NOTE: The drive motor control and joystick logic has been moved to CANDriveSubsystem
// and a new TeleopDriveCommand. This class is now cleaner and focused on the main
// robot lifecycle, following best practices for Command-Based programming.
public class Robot extends TimedRobot {
  
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    // This creates HttpCamera objects for each Limelight and then explicitly
    // tells the CameraServer to start capturing and streaming them.

    // Create a camera object for the right Limelight
    HttpCamera limelightRight = new HttpCamera("limelight-right", "http://limelight-right.local:5800/stream.mjpg");
    // Tell the CameraServer to start streaming this camera
    CameraServer.startAutomaticCapture(limelightRight);

    // Create a camera object for the left Limelight
    HttpCamera limelightLeft = new HttpCamera("limelight-left", "http://limelight-left.local:5801/stream.mjpg");
    // Tell the CameraServer to start streaming this camera
    CameraServer.startAutomaticCapture(limelightLeft);
  
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

  @Override
  public void teleopPeriodic() {
    // Teleop logic is now handled by the command scheduler.
    // The default TeleopDriveCommand will run automatically.
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

