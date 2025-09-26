// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Map;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double CONTROLLER_DEADZONE = 0.2;
  }

  public static class PIDConstants {
    public static final double P = 0.1;
    public static final double I = 0.0;
    public static final double D = 0.0;
    
    // TODO: Tune Auto-Align PID constant
    public static final double kP_AUTO_ALIGN = 0.05; // P gain for auto-align rotation
  }

  public static class PIDElevatorConstants {
    // TODO: Tune Elevator PID constants
    public static final double P = 0.1;
    public static final double I = 0.0;
    public static final double D = 0.0;
  }
  public static final class ClimberConstants {
    public static final double MAX_BOUND = 100.0;
    public static final double MIN_BOUND = -10.0;
    public static final int LEFT_CLIMBER_ID = 27;
    public static final int RIGHT_CLIMBER_ID = 24;
  }

    public static final class DriveConstants {
    public static final int LEFT_LEADER_ID = 8;
    public static final int LEFT_FOLLOWER_ID = 9;
    public static final int RIGHT_LEADER_ID = 7;
    public static final int RIGHT_FOLLOWER_ID = 6;
  }

  public static final class ElevatorConstants {
    public static final int ELEVATOR_LEADER_ID = 2;
    public static final int ELEVATOR_FOLLOWER_ID = 4;

    // TODO: Tune Elevator PID gains for position control
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // TODO: Tune Min/Max output for Elevator PID controller
    public static final double kMIN_OUTPUT = -0.7;
    public static final double kMAX_OUTPUT = 0.7;

    // TODO: Tune tolerance for Elevator atSetpoint() check
    public static final double kPOSITION_TOLERANCE = 0.5;

    // TODO: Calibrate Elevator preset positions in motor rotations
    // Based on calculations: L2 = 16.85in, L3 = 32.54in
    public static final double L2_POSITION_ROTATIONS = 16.8; 
    public static final double L3_POSITION_ROTATIONS = 32.5;

    
    public static final double MANUAL_RAISE_SPEED = 0.4;
    public static final double MANUAL_LOWER_SPEED = -0.3;
  }
  


  public static final class AlgaeArmConstants {
    public static final int ALGAE_ARM_ID = 21;
    public static final double ALGAE_ARM_UP_POSITION = -3.3;
    public static final double ALGAE_ARM_DOWN_POSITION = -7.4;
  }

  public static final class AlgaeIntakeConstants {
    public static final int ALGAE_INTAKE_ID = 11;
    public static final double ALGAE_INTAKE_SPEED = 0.5;
  }

  public static final class ArmConstants {
    public static final int CORAL_ARM_ID = 3;

    // TODO: Tune Arm PID gains for position control
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // TODO: Tune Min/Max output for Arm PID controller
    public static final double kMIN_OUTPUT = -0.5;
    public static final double kMAX_OUTPUT = 0.5;

    // TODO: Tune tolerance for Arm atSetpoint() check
    public static final double kPOSITION_TOLERANCE = 0.5;
    
    // TODO: Calibrate Arm preset positions in motor rotations
    // Based on calculations: L2 = 53.44deg, L3 = 54.20deg
    public static final double L2_POSITION_ROTATIONS = 26.7; 
    public static final double L3_POSITION_ROTATIONS = 27.1;

    // TODO: Tune manual arm speed
    public static final double MANUAL_ARM_SPEED = 0.4;
  }

  public static final class CoralIntakeConstants {
    public static final int CORAL_INTAKE_ID = 16;
    public static final double CORAL_INTAKE_SPEED = 0.5;
  }
  
  public static final class LedConstants {
    public static final int LED_PORT = 5;
    public static final int LED_LENGTH = 142;
  }

  public static final class VisionConstants {
    // TODO: REPLACE WITH YOUR ROBOT'S ACTUAL MEASUREMENTS
    /**
     * The height of the LEFT Limelight lens from the floor in meters.
     */
    public static final double LEFT_CAMERA_HEIGHT_METERS = Units.inchesToMeters(24); 
    public static final double LEFT_CAMERA_PITCH_RADIANS = 0.0;

    public static final double RIGHT_CAMERA_HEIGHT_METERS = Units.inchesToMeters(26); 
    public static final double RIGHT_CAMERA_PITCH_RADIANS = 0.0;


    /**
     * A list of AprilTag IDs that are valid targets for scoring CORAL.
     * These are the tags located on the REEF structures.
     */
    public static final List<Integer> CORAL_SCORING_TAG_IDS = List.of(
      // Red Alliance REEF Tags
      9, 10, 11,
      // Blue Alliance REEF Tags
      13, 14, 15
    );
  }

  public static final class AutoConstants {
    // TODO: Tune autonomous command values
    public static final double AUTO_DRIVE_SPEED = 0.25;
    public static final double AUTO_DRIVE_SECONDS = 3.0;
    public static final double AUTO_INTAKE_SPEED = 0.2;
  }

  public static final class FieldConstants {
    // Official AprilTag locations from the 2025 Game Manual (v1.1, 9/19/2024)
    // All measurements are relative to the Blue Alliance origin.
    public static final Map<Integer, Pose3d> APRIL_TAG_FIELD_LAYOUT = Map.ofEntries(
        Map.entry(1, new Pose3d(15.5138, 1.0716, 0.4628, new Rotation3d(0, 0, Units.degreesToRadians(120)))),
        Map.entry(2, new Pose3d(16.204, 3.002, 0.6955, new Rotation3d(0, 0, Units.degreesToRadians(180)))),
        Map.entry(3, new Pose3d(16.204, 4.983, 0.6955, new Rotation3d(0, 0, Units.degreesToRadians(180)))),
        Map.entry(4, new Pose3d(15.5138, 6.9144, 0.4628, new Rotation3d(0, 0, Units.degreesToRadians(240)))),
        Map.entry(5, new Pose3d(0.9964, 1.0716, 0.4628, new Rotation3d(0, 0, Units.degreesToRadians(60)))),
        Map.entry(6, new Pose3d(0.306, 3.002, 0.6955, new Rotation3d(0, 0, 0))),
        Map.entry(7, new Pose3d(0.306, 4.983, 0.6955, new Rotation3d(0, 0, 0))),
        Map.entry(8, new Pose3d(0.9964, 6.9144, 0.4628, new Rotation3d(0, 0, Units.degreesToRadians(300)))),
        Map.entry(9, new Pose3d(13.5905, 7.4994, 0.4636, new Rotation3d(0, 0, 0))),
        Map.entry(10, new Pose3d(14.6827, 5.5055, 0.4636, new Rotation3d(0, 0, 0))),
        Map.entry(11, new Pose3d(13.5905, 3.5116, 0.4636, new Rotation3d(0, 0, 0))),
        Map.entry(12, new Pose3d(16.1671, 3.5116, 0.6096, new Rotation3d(0, 0, Units.degreesToRadians(60)))),
        Map.entry(13, new Pose3d(2.9195, 7.4994, 0.4636, new Rotation3d(0, 0, Units.degreesToRadians(180)))),
        Map.entry(14, new Pose3d(1.8273, 5.5055, 0.4636, new Rotation3d(0, 0, Units.degreesToRadians(180)))),
        Map.entry(15, new Pose3d(2.9195, 3.5116, 0.4636, new Rotation3d(0, 0, Units.degreesToRadians(180)))),
        Map.entry(16, new Pose3d(0.3429, 3.5116, 0.6096, new Rotation3d(0, 0, Units.degreesToRadians(120)))),
        Map.entry(17, new Pose3d(10.2934, 3.9942, 0.8733, new Rotation3d(0, 0, Units.degreesToRadians(180)))),
        Map.entry(18, new Pose3d(8.3058, 7.728, 0.6096, new Rotation3d(0, 0, Units.degreesToRadians(90)))),
        Map.entry(19, new Pose3d(8.3058, 0.2604, 0.6096, new Rotation3d(0, 0, Units.degreesToRadians(270)))),
        Map.entry(20, new Pose3d(6.2167, 3.9942, 0.8733, new Rotation3d(0, 0, 0))),
        Map.entry(21, new Pose3d(8.2042, 7.728, 0.6096, new Rotation3d(0, 0, Units.degreesToRadians(90)))),
        Map.entry(22, new Pose3d(8.2042, 0.2604, 0.6096, new Rotation3d(0, 0, Units.degreesToRadians(270))))
    );
  }
}

