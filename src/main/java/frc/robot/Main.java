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
    // Use unique Driver Station ports to avoid conflicts.

    // --- Limelight Left (Assuming Swapped Ports: Web=5801, Stream=5800) ---
    // Forward DS port 5800 (standard web access) -> LL-Left port 5801 (actual web port)
    PortForwarder.add(5800, "limelight-left.local", 5801);
    // Forward DS port 5801 (standard stream access) -> LL-Left port 5800 (actual stream port)
    PortForwarder.add(5801, "limelight-left.local", 5800);

    // --- Limelight Right (Assuming Default Ports: Web=5800, Stream=5801) ---
    // Forward DS port 5802 (unique for LL-Right web) -> LL-Right port 5800 (default web port)
    PortForwarder.add(5802, "limelight-right.local", 5800);
    // Forward DS port 5803 (unique for LL-Right stream) -> LL-Right port 5801 (default stream port)
    PortForwarder.add(5803, "limelight-right.local", 5801);

    // Optional: Forward other ports if needed, using unique DS ports
    // PortForwarder.add(5804, "limelight-left.local", 5802); // Example if LL-Left used 5802
    // PortForwarder.add(5805, "limelight-right.local", 5805); // Example if LL-Right used 5805

    // Start the robot code
    RobotBase.startRobot(Robot::new);
  }
}

