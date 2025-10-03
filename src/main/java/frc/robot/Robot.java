// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// THE FIX: Import the CameraServer class
import edu.wpi.first.cameraserver.CameraServer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OperatorConstants;

public class Robot extends TimedRobot {
  
  private final CANBus kCANBus = new CANBus();

  public final TalonFX leftLeader = new TalonFX(8, kCANBus);
  public final TalonFX leftFollower = new TalonFX(9, kCANBus);
  public final TalonFX rightLeader = new TalonFX(7, kCANBus);
  public final TalonFX rightFollower = new TalonFX(6, kCANBus);

  public final DutyCycleOut leftOut = new DutyCycleOut(0);
  public final DutyCycleOut rightOut = new DutyCycleOut(0);

  public int printCount = 0;

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private final XboxController joystick = new XboxController(0);

  @Override
  public void robotInit() {
    // THE FIX: Explicitly start a camera stream for each Limelight.
    // This gives them unique names and tells the CameraServer to expect two cameras.
    // The numbers 0 and 1 correspond to the USB ports on the RoboRIO (/dev/video0, /dev/video1).
    CameraServer.startAutomaticCapture("limelight-right", 0);
    CameraServer.startAutomaticCapture("limelight-left", 1);


    var leftConfiguration = new TalonFXConfiguration();
    var rightConfiguration = new TalonFXConfiguration();

    /* User can optionally change the configs or leave it alone to perform a factory default */
    leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    leftLeader.getConfigurator().apply(leftConfiguration);
    leftFollower.getConfigurator().apply(leftConfiguration);
    rightLeader.getConfigurator().apply(rightConfiguration);
    rightFollower.getConfigurator().apply(rightConfiguration);

    /* Set up followers to follow leaders */
    leftFollower.setControl(new Follower(leftLeader.getDeviceID(), false));
    rightFollower.setControl(new Follower(rightLeader.getDeviceID(), false));
  
    leftLeader.setSafetyEnabled(true);
    rightLeader.setSafetyEnabled(true);

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
    double fwd = joystick.getLeftY(); 
    double rot = -(joystick.getRightX());
    double deadzone = OperatorConstants.CONTROLLER_DEADZONE;

    if (fwd >= -deadzone && fwd <= deadzone) {
      fwd = 0;
    }
    if (rot >= -deadzone && rot <= deadzone) {
      rot = 0;
    }

    leftOut.Output = (fwd + rot)*.65;
    rightOut.Output = (fwd - rot)*.65;

    if (!joystick.getAButton()) {
      leftLeader.setControl(leftOut);
      rightLeader.setControl(rightOut);
    }
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
  }

  @Override
  public void simulationPeriodic() {
    
  }
  
}
