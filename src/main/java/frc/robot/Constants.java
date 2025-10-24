// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AutoAlignCommand.AlignMode;
// Removed unused import
// import frc.robot.commands.SetScoringPositionCommand.ScoringLevel;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final double CONTROLLER_DEADZONE = 0.2;
    }

    public static final class DriveConstants {
        public static final int LEFT_LEADER_ID = 8;
        public static final int LEFT_FOLLOWER_ID = 9;
        public static final int RIGHT_LEADER_ID = 7;
        public static final int RIGHT_FOLLOWER_ID = 6;
        public static final double DRIVE_GEARING = 8.26; // TODO: Verify drive gearing ratio
        public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(3.0); // TODO: Verify wheel radius
        public static final double TRACK_WIDTH_METERS = 0.65; // TODO: Verify track width measurement

        public static final double WHEEL_CIRCUMFERENCE_METERS = 2 * Math.PI * WHEEL_RADIUS_METERS;
        // Conversion factor from motor rotations to linear distance (meters)
        public static final double ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE_METERS; // /DriveGEar


        // --- Trajectory Following Constants ---
        // TODO: Characterize the drivetrain using SysId to get accurate values.
        public static final double kS = 0.1332; // Volts
        public static final double kV = 2.1615; // Volt-seconds per meter
        public static final double kA = 0.2825; // Volt-seconds-squared per meter

        // TODO: Tune these PID gains for the TalonFX's internal velocity controller, especially if using Ramsete or Motion Magic.
        public static final double kP_VELOCITY = 0.1;
        public static final double kI_VELOCITY = 0.0;
        public static final double kD_VELOCITY = 0.0;

        // --- Robot Speed Limits ---
        // TODO: Tune these values based on robot performance and testing.
        public static final double MAX_SPEED_MPS = 3.0; // Meters per second
        public static final double MAX_ACCELERATION_MPS_SQ = 2.0; // Meters per second squared
        public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = Math.PI * 1.5; // Radians per second

        // Ramsete Controller parameters (defaults are usually okay)
        public static final double RAMSETE_B = 2.0;
        public static final double RAMSETE_ZETA = 0.7;
    }

    public static class PIDConstants { // TODO: Review if these generic constants are still needed or should be specific
        public static final double P = 0.1;
        public static final double I = 0.0;
        public static final double D = 0.0;
    }

    public static class VelocityPIDConstants { // TODO: Review if these generic constants are still needed or should be specific
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public static class PIDElevatorConstants { // TODO: Review if these generic constants are still needed or should be specific
        public static final double P = 0.1;
        public static final double I = 0.0;
        public static final double D = 0.0;
    }

    public static final class ClimberConstants {
        // TODO: Tune these software bounds to prevent mechanical damage.
        public static final double MAX_BOUND = 100.0; // Example upper limit
        public static final double MIN_BOUND = -10.0; // Example lower limit
        public static final int LEFT_CLIMBER_ID = 27;
        public static final int RIGHT_CLIMBER_ID = 24;
    }

    public static final class AutoAlignConstants {
        // --- PID Constants (Not currently used by trajectory AutoAlignCommand) ---
        // TODO: Tune these if using a PID-based alignment command again.
        public static final double kP_TURN = 0.6;
        public static final double kI_TURN = 0.1;
        public static final double kD_TURN = 0.1;
        public static final double TURN_TOLERANCE_DEGREES = 2.0;

        public static final double kP_DRIVE = 1.0;
        public static final double kI_DRIVE = 0.0;
        public static final double kD_DRIVE = 0.0;
        public static final double DRIVE_TOLERANCE_METERS = 0.1;

        // --- Trajectory Alignment Constants ---
        // TODO: Tune this distance for optimal approach behavior.
        public static final double APPROACH_DISTANCE_METERS = 1.0; // Distance back from final stop

        // TODO: Tune this distance for the final scoring position relative to the pipe base.
        public static final double DESIRED_STOP_DISTANCE_METERS = 0.5; // Distance from pipe base

        // TODO: Tune these weights for the target cost calculation if needed.
        public static final double DISTANCE_WEIGHT = 1.5;
        public static final double DRIVE_DIRECTION_WEIGHT = 2.0;
        public static final double REEF_STATE_WEIGHT = 100.0;
    }
    public static final class ElevatorConstants {
        public static final int ELEVATOR_LEADER_ID = 2;
        public static final int ELEVATOR_FOLLOWER_ID = 4;

        // TODO: Tune Elevator PID gains for position control
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.3; // Feedforward - tune based on gravity load
        public static final double kMIN_OUTPUT = -0.7; // TODO: Tune max speeds
        public static final double kMAX_OUTPUT = 0.7;
        public static final double kPOSITION_TOLERANCE = 0.5; // Rotations - tune based on acceptable error
        // TODO: Tune these preset positions based on encoder readings at desired heights.
        public static final double L2_POSITION_ROTATIONS = 27.3;
        public static final double L3_POSITION_ROTATIONS = 44.02;
        public static final double LOADING_POSITION_ROTATIONS = 57.6;
        // TODO: Tune manual control speeds.
        public static final double MANUAL_RAISE_SPEED = 0.4;
        public static final double MANUAL_LOWER_SPEED = -0.3;
    }
    public static final class AlgaeArmConstants {
        public static final int ALGAE_ARM_ID = 21;
        // TODO: Tune these preset positions based on encoder readings.
        public static final double ALGAE_ARM_UP_POSITION = -3.3;
        public static final double ALGAE_ARM_DOWN_POSITION = -7.4;
    }
    public static final class AlgaeIntakeConstants {
        public static final int ALGAE_INTAKE_ID = 11;
        // TODO: Tune intake speed.
        public static final double ALGAE_INTAKE_SPEED = 0.5;
    }
    public static final class ArmConstants {
        public static final int CORAL_ARM_ID = 3;

        // TODO: Tune Arm PID gains for position control
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.2; // Feedforward - tune based on gravity/load
        public static final double kMIN_OUTPUT = -0.5; // TODO: Tune max speeds
        public static final double kMAX_OUTPUT = 0.5;
        public static final double kPOSITION_TOLERANCE = 0.5; // Rotations - tune based on acceptable error
        // TODO: Tune these preset positions based on encoder readings.
        public static final double L2_POSITION_ROTATIONS = -15;
        public static final double L3_POSITION_ROTATIONS = -13;
        public static final double HOME_POSITION_ROTATIONS = 0.0; // Assumed zero point
        public static final double LOADING_POSITION_ROTATIONS = 50.0;
        // TODO: Tune manual control speed.
        public static final double MANUAL_ARM_SPEED = 0.5;
    }
    public static final class CoralIntakeConstants {
        public static final int CORAL_INTAKE_ID = 16;
        // TODO: Tune intake speed.
        public static final double CORAL_INTAKE_SPEED = 0.6;
    }
    public static final class LedConstants {
        public static final int LED_PORT = 5; // TODO: Verify PWM port
        public static final int LED_LENGTH = 142; // TODO: Verify LED strip length
    }
    public static final class VisionConstants {

        // --- NEW: Fixed Pipe Poses ---
        /** Represents fixed base poses for a scoring pipe (Blue and Red alliance perspectives). */
        public record PipeBase(String name, Pose2d blue, Pose2d red) {}

        /** List of predefined pipe base poses from the supplemental file. TODO: VERIFY THESE COORDINATES against field measurements. */
        public static final List<PipeBase> PIPES = List.of(
            new PipeBase("A", new Pose2d(3.71, 4.19, Rotation2d.fromDegrees(0)), new Pose2d(13.84, 3.86, Rotation2d.fromDegrees(180))),
            new PipeBase("B", new Pose2d(3.71, 3.86, Rotation2d.fromDegrees(0)), new Pose2d(13.84, 4.19, Rotation2d.fromDegrees(180))),
            new PipeBase("C", new Pose2d(3.96, 3.43, Rotation2d.fromDegrees(60)), new Pose2d(13.59, 4.62, Rotation2d.fromDegrees(240))),
            new PipeBase("D", new Pose2d(4.24, 3.27, Rotation2d.fromDegrees(60)), new Pose2d(13.31, 4.78, Rotation2d.fromDegrees(240))),
            new PipeBase("E", new Pose2d(4.74, 3.27, Rotation2d.fromDegrees(120)), new Pose2d(12.81, 4.78, Rotation2d.fromDegrees(300))),
            new PipeBase("F", new Pose2d(5.02, 3.43, Rotation2d.fromDegrees(120)), new Pose2d(12.53, 4.62, Rotation2d.fromDegrees(300))),
            new PipeBase("G", new Pose2d(5.27, 3.86, Rotation2d.fromDegrees(180)), new Pose2d(12.29, 4.19, Rotation2d.fromDegrees(0))),
            new PipeBase("H", new Pose2d(5.27, 4.19, Rotation2d.fromDegrees(180)), new Pose2d(12.29, 3.86, Rotation2d.fromDegrees(0))),
            new PipeBase("I", new Pose2d(5.02, 4.62, Rotation2d.fromDegrees(240)), new Pose2d(12.53, 3.43, Rotation2d.fromDegrees(60))),
            new PipeBase("J", new Pose2d(4.74, 4.78, Rotation2d.fromDegrees(240)), new Pose2d(12.81, 3.27, Rotation2d.fromDegrees(60))),
            new PipeBase("K", new Pose2d(4.24, 4.78, Rotation2d.fromDegrees(300)), new Pose2d(13.31, 3.27, Rotation2d.fromDegrees(120))),
            new PipeBase("L", new Pose2d(3.96, 4.62, Rotation2d.fromDegrees(300)), new Pose2d(13.59, 3.43, Rotation2d.fromDegrees(120)))
        );
        // --- End Fixed Pipe Poses ---

        /**
         * Represents the information needed to identify a scoring target.
         * Includes the dynamically calculated approach and final scoring poses relative to a fixed base pose.
         */
        public static class ScoringTargetInfo {
            // No longer storing parentTagId directly here, rely on PipeBase name
            public final String id; // Unique ID for this specific pipe (e.g., "A", "B")
            public final Pose2d approachPose;
            public final Pose2d finalPose;
            public final PipeBase pipeBase; // Store the reference pipe base

            public ScoringTargetInfo(String id, Pose2d approachPose, Pose2d finalPose, PipeBase pipeBase) {
                this.id = id;
                this.approachPose = approachPose;
                this.finalPose = finalPose;
                this.pipeBase = pipeBase;
            }
        }

        // TODO: VERIFY THESE CAMERA TRANSFORMS ON THE ROBOT using measurements from the robot's center.
        // Camera transforms relative to robot center (X=forward, Y=left, Z=up)
        public static final Transform3d LEFT_ROBOT_TO_CAMERA = new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(6), Units.inchesToMeters(10)), // Example
            new Rotation3d(0.0, Units.degreesToRadians(-15), 0.0)); // Example
        public static final Transform3d RIGHT_ROBOT_TO_CAMERA = new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(-6), Units.inchesToMeters(10)), // Example
            new Rotation3d(0.0, Units.degreesToRadians(-15), 0.0)); // Example


        // Robot's desired final orientation RELATIVE TO FIELD when scoring/collecting.
        // TODO: VERIFY these orientations based on mechanism placement.
        public static final Rotation2d SCORING_ORIENTATION_FIELD = Rotation2d.fromDegrees(180.0); // Facing own alliance wall
        public static final Rotation2d COLLECTING_ORIENTATION_FIELD = Rotation2d.fromDegrees(0.0); // Facing opponent alliance wall

        // List of AprilTag IDs associated with coral scoring locations used for VISIBILITY CHECK.
        // This list does NOT directly determine the target pose anymore.
        // TODO: Ensure this list includes tags relevant to the PIPES defined above.
         public static final List<Integer> VALID_SCORING_TAG_IDS = List.of(6, 7 /* Add other relevant tags here: 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 */);


        /**
         * Calculates the desired field-relative robot poses (approach and final) relative to a FIXED pipe base pose.
         *
         * @param pipeBase The fixed PipeBase object containing Blue and Red poses.
         * @param alliance The current alliance color.
         * @param mode The alignment mode (SCORING or COLLECTING), which determines robot orientation.
         * @return An Optional containing the ScoringTargetInfo (including approach and final poses calculated relative to the fixed pipe base).
         */
        public static Optional<ScoringTargetInfo> getScoringTargetInfo(PipeBase pipeBase, Alliance alliance, AlignMode mode) {

             // 1. Select the correct fixed base pose based on alliance
             Pose2d basePipePose = (alliance == Alliance.Red) ? pipeBase.red() : pipeBase.blue();

            // 2. Determine the final robot orientation based on the alignment mode and alliance.
            // For fixed pipe poses, the orientation should generally match the pipe's defined orientation.
            Rotation2d targetRobotFinalOrientation = basePipePose.getRotation();

            // Transform to get from the Pipe Base pose center to the Robot center at the final stop point.
            // Robot's X-axis is aligned with Pipe Base's X-axis, offset by desired distance.
            // ASSUMES ROBOT FRONT FACES PIPE BASE (+X towards pipe base)
            Transform2d pipeToFinalStopRobot = new Transform2d(
                new Translation2d(-AutoAlignConstants.DESIRED_STOP_DISTANCE_METERS, 0), // Move back along Pipe Base's -X axis
                Rotation2d.fromDegrees(0) // Align robot rotation with pipe base rotation
            );

            // Calculate the final pose by applying the transform to the pipe base pose
            Pose2d finalPose = basePipePose.plus(pipeToFinalStopRobot);
            // Ensure the final pose has the correct absolute field orientation from the pipe base
            finalPose = new Pose2d(finalPose.getTranslation(), targetRobotFinalOrientation);


            // Transform to get from the Final Stop pose to the Approach pose (further back)
            // This transform is relative to the ROBOT's frame at the final stop pose.
            Transform2d finalToApproachRobot = new Transform2d(
                new Translation2d(-AutoAlignConstants.APPROACH_DISTANCE_METERS, 0), // Move back along the ROBOT's -X axis
                Rotation2d.fromDegrees(0) // No change in rotation
            );

            // 5. Calculate the approach pose based on the final pose
            Pose2d approachPose = finalPose.plus(finalToApproachRobot);
            // Ensure approach pose also has the correct final orientation
            approachPose = new Pose2d(approachPose.getTranslation(), targetRobotFinalOrientation);

            // 6. Construct the ID (using PipeBase name) and return the result
            return Optional.of(new ScoringTargetInfo(pipeBase.name(), approachPose, finalPose, pipeBase));
        }

         /**
          * Gets a list of all potential ScoringTargetInfo objects based on the fixed PIPES list.
          * @param alliance Current alliance
          * @param mode Alignment mode (SCORING/COLLECTING)
          * @return List of ScoringTargetInfo
          */
         public static List<ScoringTargetInfo> getAllPotentialTargets(Alliance alliance, AlignMode mode) {
            return PIPES.stream()
                .map(pipe -> getScoringTargetInfo(pipe, alliance, mode))
                .filter(Optional::isPresent)
                .map(Optional::get)
                .collect(Collectors.toList());
        }
    }


    // Constants for simple time-based auto
    public static final class AutoConstants {
        // TODO: Tune these speeds and times.
        public static final double AUTO_DRIVE_SPEED = 0.25; // Percent output
        public static final double AUTO_DRIVE_SECONDS = 3.0;
        public static final double AUTO_INTAKE_SPEED = 0.2; // Percent output (placeholder)

        // AprilTag IDs for coral collection zones
        // TODO: Verify these IDs based on the 2025 field layout.
        public static final List<Integer> RED_CORAL_COLLECTION_TAG_IDS = List.of(1, 2);
        public static final List<Integer> BLUE_CORAL_COLLECTION_TAG_IDS = List.of(12, 13); // Example IDs

        // PID constants for DriveToPoseCommand (used if NOT using trajectories)
        // TODO: Tune these PID values if using DriveToPoseCommand.
        public static final double kP_DRIVE_TO_POSE = 1.2;
        public static final double kP_TURN_TO_POSE = 0.05;
        public static final double DRIVE_TO_POSE_TOLERANCE_METERS = 0.05; // 5 cm
        public static final double TURN_TO_POSE_TOLERANCE_DEGREES = 2.0; // 2 degrees
    }

    public static final class FieldConstants {
        public static final double FIELD_LENGTH_METERS = Units.feetToMeters(54.17); // 16.51
        public static final double FIELD_WIDTH_METERS = Units.feetToMeters(26.93); // 8.21

        // Official AprilTag field layout for REEFSCAPE 2025
        // TODO: Ensure this map is up-to-date with the official layout for 2025.
         public static final Map<Integer, Pose3d> APRIL_TAG_FIELD_LAYOUT = Map.ofEntries(
            Map.entry(1, new Pose3d(15.5138, 1.0716, 0.4628, new Rotation3d(0, 0, Units.degreesToRadians(120)))),
            Map.entry(2, new Pose3d(16.204, 3.002, 0.6955, new Rotation3d(0, 0, Units.degreesToRadians(180)))),
            Map.entry(3, new Pose3d(16.204, 4.983, 0.6955, new Rotation3d(0, 0, Units.degreesToRadians(180)))),
            Map.entry(4, new Pose3d(15.5138, 6.9144, 0.4628, new Rotation3d(0, 0, Units.degreesToRadians(240)))),
            Map.entry(5, new Pose3d(0.9964, 1.0716, 0.4628, new Rotation3d(0, 0, Units.degreesToRadians(60)))),
            Map.entry(6, new Pose3d(0.306, 3.002, 0.6955, new Rotation3d(0, 0, 0))), // Blue Source Side
            Map.entry(7, new Pose3d(0.306, 4.983, 0.6955, new Rotation3d(0, 0, 0))), // Blue Amp Side
            Map.entry(8, new Pose3d(0.9964, 6.9144, 0.4628, new Rotation3d(0, 0, Units.degreesToRadians(300)))),
            Map.entry(9, new Pose3d(13.5905, 7.4994, 0.4636, new Rotation3d(0, 0, Units.degreesToRadians(180)))), // Red Stage Far
            Map.entry(10, new Pose3d(14.6827, 5.5055, 0.4636, new Rotation3d(0, 0, Units.degreesToRadians(180)))), // Red Stage Center
            Map.entry(11, new Pose3d(13.5905, 3.5116, 0.4636, new Rotation3d(0, 0, Units.degreesToRadians(180)))), // Red Stage Close
            Map.entry(12, new Pose3d(16.1671, 3.5116, 0.6096, new Rotation3d(0, 0, Units.degreesToRadians(60)))), // Red Source Station Lower
            Map.entry(13, new Pose3d(2.9195, 7.4994, 0.4636, new Rotation3d(0, 0, 0))), // Blue Stage Far
            Map.entry(14, new Pose3d(1.8273, 5.5055, 0.4636, new Rotation3d(0, 0, 0))), // Blue Stage Center
            Map.entry(15, new Pose3d(2.9195, 3.5116, 0.4636, new Rotation3d(0, 0, 0))), // Blue Stage Close
            Map.entry(16, new Pose3d(0.3429, 3.5116, 0.6096, new Rotation3d(0, 0, Units.degreesToRadians(120)))), // Blue Source Station Lower
            Map.entry(17, new Pose3d(10.2934, 3.9942, 0.8733, new Rotation3d(0, 0, Units.degreesToRadians(0)))), // Red Center Reef Upper
            Map.entry(18, new Pose3d(8.3058, 7.728, 0.6096, new Rotation3d(0, 0, Units.degreesToRadians(270)))), // Center Reef Barrier Far Red Side
            Map.entry(19, new Pose3d(8.3058, 0.2604, 0.6096, new Rotation3d(0, 0, Units.degreesToRadians(90)))), // Center Reef Barrier Close Red Side
            Map.entry(20, new Pose3d(6.2167, 3.9942, 0.8733, new Rotation3d(0, 0, Units.degreesToRadians(180)))), // Blue Center Reef Upper
            Map.entry(21, new Pose3d(8.2042, 7.728, 0.6096, new Rotation3d(0, 0, Units.degreesToRadians(270)))), // Center Reef Barrier Far Blue Side
            Map.entry(22, new Pose3d(8.2042, 0.2604, 0.6096, new Rotation3d(0, 0, Units.degreesToRadians(90)))) // Center Reef Barrier Close Blue Side
        );
    }
}

