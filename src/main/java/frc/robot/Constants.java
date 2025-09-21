// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
}

