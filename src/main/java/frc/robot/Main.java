// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    // --- Add Port Forwarding for Limelights ---
    // --- NOTE: Ports are swapped based on user observation (Stream=5800, Web=5801) ---

    // Forward Driver Station port 5800 (standard web) to Limelight port 5801 (actual web)
    PortForwarder.add(5800, "limelight-left.local", 5801);
    PortForwarder.add(5800, "limelight-right.local", 5801);

    // Forward Driver Station port 5801 (standard stream) to Limelight port 5800 (actual stream)
    PortForwarder.add(5801, "limelight-left.local", 5800);
    PortForwarder.add(5801, "limelight-right.local", 5800);

    // Forward other potential Limelight ports (less critical, but good practice)
    PortForwarder.add(5802, "limelight-left.local", 5802);
    PortForwarder.add(5802, "limelight-right.local", 5802);
    PortForwarder.add(5803, "limelight-left.local", 5803);
    PortForwarder.add(5803, "limelight-right.local", 5803);
    PortForwarder.add(5804, "limelight-left.local", 5804);
    PortForwarder.add(5804, "limelight-right.local", 5804);
    PortForwarder.add(5805, "limelight-left.local", 5805);
    PortForwarder.add(5805, "limelight-right.local", 5805);

    // Start the robot code
    RobotBase.startRobot(Robot::new);
  }
}

