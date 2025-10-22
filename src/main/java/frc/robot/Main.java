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
    // --- Port Forwarding REMOVED for Testing ---
    // Relying on Limelight to auto-publish stream URL to NetworkTables
    // and dashboard to read it from there.

    System.out.println("Port Forwarding Disabled for Testing.");

    /*
    // --- Add Port Forwarding for Limelight Left ONLY ---
    // Focus on the USB connection at 172.28.0.1
    // Assuming Swapped Ports: Web=5801, Stream=5800

    System.out.println("Setting up Port Forwarding for Limelight-Left (USB)...");

    // Forward DS port 5800 (standard web access) -> LL-Left IP:5801 (actual web port)
    try {
        PortForwarder.add(5800, "172.28.0.1", 5801);
        System.out.println("Forwarding DS 5800 -> 172.28.0.1:5801 (Web Interface)");
    } catch (Exception e) {
        System.err.println("Error adding port forward 5800->5801: " + e.getMessage());
    }

    // Forward DS port 5801 (standard stream access) -> LL-Left IP:5800 (actual stream port)
     try {
        PortForwarder.add(5801, "172.28.0.1", 5800);
        System.out.println("Forwarding DS 5801 -> 172.28.0.1:5800 (Stream)");
    } catch (Exception e) {
        System.err.println("Error adding port forward 5801->5800: " + e.getMessage());
    }


    // --- Limelight Right Forwarding REMOVED for testing ---
    // PortForwarder.add(5802, "limelight-right.local", 5800);
    // PortForwarder.add(5803, "limelight-right.local", 5801);

    System.out.println("Port forwarding setup complete.");
    */

    // Start the robot code
    RobotBase.startRobot(Robot::new);
  }
}

