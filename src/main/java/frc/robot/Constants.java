// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.SetScoringPositionCommand.ScoringLevel;

/**

The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
constants. This class should not be used for any other purpose. All constants should be declared
globally (i.e. public static). Do not put anything functional in this class.
It is advised to statically import this class (or one of its inner classes) wherever the

constants are needed, to reduce verbosity.
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
    }

    public static class PIDElevatorConstants {
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
    
        // --- TRAJECTORY FOLLOWING CONSTANTS ---
        // These are example values and MUST be tuned for your robot.
        public static final double kS = 0.1;   // Volts
        public static final double kV = 2.0;   // Volts * seconds / meter
        public static final double kA = 0.2;   // Volts * seconds^2 / meter
    
        // These are PID gains for the TalonFX's internal velocity control.
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    
        // --- PHYSICAL CHARACTERISTICS ---
        public static final double DRIVE_GEARING = 8.45; // e.g., 8.45:1
        public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(3.0);
        public static final double WHEEL_CIRCUMFERENCE_M = 2 * Math.PI * WHEEL_RADIUS_METERS;
        public static final double TRACK_WIDTH_METERS = 0.6; // Adjust to your robot's track width
    
        // --- PERFORMANCE ---
        public static final double CRUISE_MPS = 2.5; // Cruise velocity in meters/sec
        public static final double ACCEL_MPS2 = 2.0; // Acceleration in meters/sec^2
        public static final double OPEN_LOOP_SCALE = 0.65; // Scale factor for teleop driving
    }

    public static final class AutoAlignConstants {
        public static final double DESIRED_DISTANCE_METERS = 0.2; // How close to get to the scoring peg
    }
    
    public static final class ElevatorConstants {
        public static final int ELEVATOR_LEADER_ID = 2;
        public static final int ELEVATOR_FOLLOWER_ID = 4;
    
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.3;
    
        public static final double kMIN_OUTPUT = -0.7;
        public static final double kMAX_OUTPUT = 0.7;
    
        public static final double kPOSITION_TOLERANCE = 0.5;
    
    
        public static final double L2_POSITION_ROTATIONS = 16.8; 
        public static final double L3_POSITION_ROTATIONS = 45.02;
        public static final double LOADING_POSITION_ROTATIONS = 50.0; 
    
    
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
    
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.2;
    
        public static final double kMIN_OUTPUT = -0.5;
        public static final double kMAX_OUTPUT = 0.5;
    
        public static final double kPOSITION_TOLERANCE = 0.5;
        public static final double L2_POSITION_ROTATIONS = -14; 
        public static final double L3_POSITION_ROTATIONS = -14;
        public static final double HOME_POSITION_ROTATIONS = 0.0;
    
        public static final double LOADING_POSITION_ROTATIONS = 50.0;
    
        public static final double MANUAL_ARM_SPEED = 0.5;
    }
    
    public static final class CoralIntakeConstants {
        public static final int CORAL_INTAKE_ID = 16;
        public static final double CORAL_INTAKE_SPEED = 0.5;
    }
    
    public static final class LedConstants {
        public static final int LED_PORT = 5;
        public static final int LED_LENGTH = 142;
    }    
}
