// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class Robot extends TimedRobot {
  
  private AddressableLED m_Led;
  private AddressableLEDBuffer m_LedBuffer;
  private final LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kBlue, Color.kDarkOrange);
  private static final Distance kLedSpacing = Meters.of(1 / 120.0);
   private final LEDPattern m_scrollingRainbow =
   gradient.scrollAtAbsoluteSpeed(MetersPerSecond.of(.6), kLedSpacing);

  double deadzone = 0.2;	//variable for amount of deadzone
  
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
    
    m_Led = new AddressableLED(5);
    m_LedBuffer = new AddressableLEDBuffer(142);
    m_Led.setLength(m_LedBuffer.getLength());

    m_Led.setData(m_LedBuffer);
    m_Led.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    m_scrollingRainbow.applyTo(m_LedBuffer);
    m_Led.setData(m_LedBuffer);
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
