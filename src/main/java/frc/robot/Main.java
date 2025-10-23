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
   * <p>If you change your main robot class, change the parameter class to the startRobot
   * call.
   */
  public static void main(String... args) {
    // --- Add Port Forwarding using specific USB IP Address ---
    // Use unique Driver Station ports.
    // Assuming DEFAULT Ports on Limelights (Web=5800, Stream=5801).
    // Use the IP address observed in NetworkTables for USB devices.

    String limelightLeftIP = "172.28.0.1"; // The IP address confirmed via NetworkTables
    // String limelightRightIP = "172.28.1.1"; // Example if right LL was also USB

    System.out.println("Setting up Port Forwarding for Limelight-Left (USB IP: " + limelightLeftIP + ")...");

    // --- Limelight Left (USB) ---
    // Forward DS port 5800 -> LL-Left USB IP:5800 (Default Web)
    try {
        PortForwarder.add(5800, limelightLeftIP, 5800);
        System.out.println("Forwarding DS 5800 -> " + limelightLeftIP + ":5800 (Web Interface)");
    } catch (Exception e) {
        System.err.println("Error adding port forward 5800->5800: " + e.getMessage());
    }

    // Forward DS port 5801 -> LL-Left USB IP:5801 (Default Stream)
     try {
        PortForwarder.add(5801, limelightLeftIP, 5801);
        System.out.println("Forwarding DS 5801 -> " + limelightLeftIP + ":5801 (Stream)");
    } catch (Exception e) {
        System.err.println("Error adding port forward 5801->5801: " + e.getMessage());
    }

    // --- Limelight Right (Assuming Ethernet + Default Ports) ---
    // Keep using hostname for Ethernet-connected devices if needed
     try {
        PortForwarder.add(5802, "limelight-right.local", 5800); // DS 5802 -> LL-Right Web (Default)
        System.out.println("Forwarding DS 5802 -> limelight-right.local:5800 (Web Interface)");
     } catch (Exception e) {
        System.err.println("Error adding port forward 5802->5800: " + e.getMessage());
     }
     try {
        PortForwarder.add(5803, "limelight-right.local", 5801); // DS 5803 -> LL-Right Stream (Default)
        System.out.println("Forwarding DS 5803 -> limelight-right.local:5801 (Stream)");
     } catch (Exception e) {
        System.err.println("Error adding port forward 5803->5801: " + e.getMessage());
     }


    System.out.println("Port forwarding setup complete.");
    // Start the robot code
    RobotBase.startRobot(Robot::new);
  }
}

