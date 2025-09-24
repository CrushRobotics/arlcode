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
  }

  public static class PIDConstants {
    public static final double P = 0.1;
    public static final double I = 0.0;
    public static final double D = 0.0;
  }

  public static class PIDElevatorConstants {
    public static final double P = 0.1;
    public static final double I = 0.0;
    public static final double D = 0.0;
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
  }
  
  public static final class ClimberConstants {
    public static final double MAX_BOUND = 100.0;
    public static final double MIN_BOUND = -10.0;
    public static final int LEFT_CLIMBER_ID = 27;
    public static final int RIGHT_CLIMBER_ID = 24;
  }

  public static final class AlgaeArmConstants {
    public static final int ALGAE_ARM_ID = 21;
  }

  public static final class AlgaeIntakeConstants {
    public static final int ALGAE_INTAKE_ID = 11;
  }

  public static final class CoralArmConstants {
    public static final int CORAL_ARM_ID = 3;
  }

  public static final class CoralIntakeConstants {
    public static final int CORAL_INTAKE_ID = 16;
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
      6, 7, 8, 9, 10, 11,
      // Blue Alliance REEF Tags
      17, 18, 19, 20, 21, 22
    );
  }

  public static final class FieldConstants {
    // Official AprilTag locations from the 2025 Game Manual (User Guide v1.1)
    // The format is (X, Y, Z) in meters and rotation in radians.
    // All measurements are relative to the Blue Alliance origin.
    public static final Map<Integer, Pose3d> APRIL_TAG_FIELD_LAYOUT = Map.ofEntries(
        // --- RED ALLIANCE TAGS (IDs 1-11) ---
        Map.entry(1, new Pose3d(new Translation3d(Units.inchesToMeters(610.77), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22)), new Rotation3d(0, 0, Units.degreesToRadians(120)))),
        Map.entry(2, new Pose3d(new Translation3d(Units.inchesToMeters(637.96), Units.inchesToMeters(118.19), Units.inchesToMeters(27.38)), new Rotation3d(0, 0, Units.degreesToRadians(180)))),
        Map.entry(3, new Pose3d(new Translation3d(Units.inchesToMeters(637.96), Units.inchesToMeters(196.19), Units.inchesToMeters(27.38)), new Rotation3d(0, 0, Units.degreesToRadians(180)))),
        Map.entry(4, new Pose3d(new Translation3d(Units.inchesToMeters(610.77), Units.inchesToMeters(272.19), Units.inchesToMeters(18.22)), new Rotation3d(0, 0, Units.degreesToRadians(240)))),
        Map.entry(5, new Pose3d(new Translation3d(Units.inchesToMeters(39.23), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22)), new Rotation3d(0, 0, Units.degreesToRadians(60)))),
        Map.entry(6, new Pose3d(new Translation3d(Units.inchesToMeters(12.04), Units.inchesToMeters(118.19), Units.inchesToMeters(27.38)), new Rotation3d(0, 0, 0))),
        Map.entry(7, new Pose3d(new Translation3d(Units.inchesToMeters(12.04), Units.inchesToMeters(196.19), Units.inchesToMeters(27.38)), new Rotation3d(0, 0, 0))),
        Map.entry(8, new Pose3d(new Translation3d(Units.inchesToMeters(39.23), Units.inchesToMeters(272.19), Units.inchesToMeters(18.22)), new Rotation3d(0, 0, Units.degreesToRadians(300)))),
        Map.entry(9, new Pose3d(new Translation3d(Units.inchesToMeters(535.06), Units.inchesToMeters(295.25), Units.inchesToMeters(18.25)), new Rotation3d(0, 0, 0))),
        Map.entry(10, new Pose3d(new Translation3d(Units.inchesToMeters(578.06), Units.inchesToMeters(216.75), Units.inchesToMeters(18.25)), new Rotation3d(0, 0, 0))),
        Map.entry(11, new Pose3d(new Translation3d(Units.inchesToMeters(535.06), Units.inchesToMeters(138.25), Units.inchesToMeters(18.25)), new Rotation3d(0, 0, 0))),

        // --- BLUE ALLIANCE TAGS (IDs 12-22) ---
        Map.entry(12, new Pose3d(new Translation3d(Units.inchesToMeters(636.5), Units.inchesToMeters(138.25), Units.inchesToMeters(24.0)), new Rotation3d(0, 0, Units.degreesToRadians(60)))),
        Map.entry(13, new Pose3d(new Translation3d(Units.inchesToMeters(114.94), Units.inchesToMeters(295.25), Units.inchesToMeters(18.25)), new Rotation3d(0, 0, Units.degreesToRadians(180)))),
        Map.entry(14, new Pose3d(new Translation3d(Units.inchesToMeters(71.94), Units.inchesToMeters(216.75), Units.inchesToMeters(18.25)), new Rotation3d(0, 0, Units.degreesToRadians(180)))),
        Map.entry(15, new Pose3d(new Translation3d(Units.inchesToMeters(114.94), Units.inchesToMeters(138.25), Units.inchesToMeters(18.25)), new Rotation3d(0, 0, Units.degreesToRadians(180)))),
        Map.entry(16, new Pose3d(new Translation3d(Units.inchesToMeters(13.5), Units.inchesToMeters(138.25), Units.inchesToMeters(24.0)), new Rotation3d(0, 0, Units.degreesToRadians(120)))),
        Map.entry(17, new Pose3d(new Translation3d(Units.inchesToMeters(405.25), Units.inchesToMeters(157.25), Units.inchesToMeters(34.38)), new Rotation3d(0, 0, Units.degreesToRadians(180)))),
        Map.entry(18, new Pose3d(new Translation3d(Units.inchesToMeters(327.00), Units.inchesToMeters(304.25), Units.inchesToMeters(24.0)), new Rotation3d(0, 0, Units.degreesToRadians(90)))),
        Map.entry(19, new Pose3d(new Translation3d(Units.inchesToMeters(327.00), Units.inchesToMeters(10.25), Units.inchesToMeters(24.0)), new Rotation3d(0, 0, Units.degreesToRadians(270)))),
        Map.entry(20, new Pose3d(new Translation3d(Units.inchesToMeters(244.75), Units.inchesToMeters(157.25), Units.inchesToMeters(34.38)), new Rotation3d(0, 0, 0))),
        Map.entry(21, new Pose3d(new Translation3d(Units.inchesToMeters(323.00), Units.inchesToMeters(304.25), Units.inchesToMeters(24.0)), new Rotation3d(0, 0, Units.degreesToRadians(90)))),
        Map.entry(22, new Pose3d(new Translation3d(Units.inchesToMeters(323.00), Units.inchesToMeters(10.25), Units.inchesToMeters(24.0)), new Rotation3d(0, 0, Units.degreesToRadians(270))))
    );
  }
}

