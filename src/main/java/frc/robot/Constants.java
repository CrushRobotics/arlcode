// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AutoAlignCommand.AlignMode;
import frc.robot.commands.SetScoringPositionCommand.ScoringLevel;

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
        public static final double DRIVE_GEARING = 8.26; 
        public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(3.0);
        public static final double TRACK_WIDTH_METERS = 0.264; 

        public static final double WHEEL_CIRCUMFERENCE_METERS = 2 * Math.PI * WHEEL_RADIUS_METERS;
        public static final double ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEARING;


        // --- Trajectory Following Constants ---
        public static final double kS = 0.1332; // Volts
        public static final double kV = 2.1615; // Volt-seconds per meter
        public static final double kA = 0.2825; // Volt-seconds-squared per meter

        // TODO: Tune these PID gains for the TalonFX's internal velocity controller.
        public static final double kP_VELOCITY = 0.1;
        public static final double kI_VELOCITY = 0.0;
        public static final double kD_VELOCITY = 0.0;
        
        // --- Robot Speed Limits ---
        // TODO: Tune these values to match your robot's capabilities.
        public static final double MAX_SPEED_MPS = 4.5; // Meters per second
        public static final double MAX_ACCELERATION_MPS_SQ = 3.0; // Meters per second squared
        public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = Math.PI * 2; // Radians per second
    }

    public static class PIDConstants {
        public static final double P = 0.1;
        public static final double I = 0.0;
        public static final double D = 0.0;
    }

    public static class VelocityPIDConstants {
    //tune if you need- prob not
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public static class PIDElevatorConstants {
        //tune if you need- prob not
        public static final double P = 0.1;
        public static final double I = 0.0;
        public static final double D = 0.0;
    }

    public static final class ClimberConstants {
        // TODO: Tune these bounds to prevent damage to the climber.
        public static final double MAX_BOUND = 100.0;
        public static final double MIN_BOUND = -10.0;
        public static final int LEFT_CLIMBER_ID = 27;
        public static final int RIGHT_CLIMBER_ID = 24;
    }

    public static final class AutoAlignConstants {
        // TODO: Tune these PID constants for the AutoAlign command
        public static final double kP_TURN = 0.6;
        public static final double kI_TURN = 0.1;
        public static final double kD_TURN = 0.1;
        public static final double TURN_TOLERANCE_DEGREES = 2.0;
        // TODO: Tune drive PID constants
        public static final double kP_DRIVE = 1.0;
        public static final double kI_DRIVE = 0.0;
        public static final double kD_DRIVE = 0.0;
        public static final double DRIVE_TOLERANCE_METERS = 0.1;

        // TODO: Tune this distance. Once the robot is closer than this to the target,
        // it will stop using vision and rely on odometry to finish the approach.
        public static final double FINAL_APPROACH_DISTANCE_METERS = 1.5;

        // TODO: Tune this distance. When farther than this, the robot will prioritize
        // pointing towards the target. When closer, it will prioritize the final angle.
        public static final double ROTATION_SWAP_DISTANCE_METERS = 2.0;

        // TODO: Tune this timeout. This is a safety to stop the robot if it's been
        // driving blind for too long.
        public static final double BLIND_SAFETY_TIMEOUT_SECONDS = 0.75;

        // TODO: Tune the desired distance the robot should be from the target when aligned.
        public static final double DESIRED_DISTANCE_METERS = 1.0;

        // TODO: Tune the weights for the alignment cost function.
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
        public static final double kF = 0.3;
        public static final double kMIN_OUTPUT = -0.7;
        public static final double kMAX_OUTPUT = 0.7;
        public static final double kPOSITION_TOLERANCE = 0.5;
        // TODO: Tune these preset positions.
        public static final double L2_POSITION_ROTATIONS = 27.3;
        public static final double L3_POSITION_ROTATIONS = 44.02;
        public static final double LOADING_POSITION_ROTATIONS = 57.6; 
        //elevator speeds
        public static final double MANUAL_RAISE_SPEED = 0.4;
        public static final double MANUAL_LOWER_SPEED = -0.3;
    }
    public static final class AlgaeArmConstants {
        public static final int ALGAE_ARM_ID = 21;
        // TODO: Tune these preset positions.
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
        public static final double kF = 0.2;
        public static final double kMIN_OUTPUT = -0.5;
        public static final double kMAX_OUTPUT = 0.5;
        public static final double kPOSITION_TOLERANCE = 0.5;
        // TODO: Tune these preset positions.
        public static final double L2_POSITION_ROTATIONS = -15;
        public static final double L3_POSITION_ROTATIONS = -13;
        public static final double HOME_POSITION_ROTATIONS = 0.0;
        public static final double LOADING_POSITION_ROTATIONS = 50.0;
        public static final double MANUAL_ARM_SPEED = 0.5;
    }
    public static final class CoralIntakeConstants {
        public static final int CORAL_INTAKE_ID = 16;
        // TODO: Tune intake speed.
        public static final double CORAL_INTAKE_SPEED = 0.6;
    }
    public static final class LedConstants {
        public static final int LED_PORT = 5;
        public static final int LED_LENGTH = 142;
    }
    public static final class VisionConstants {
        /**
         * Represents the information needed to identify a scoring target.
         * The actual Pose2d is calculated dynamically based on alliance.
         */
        public static class ScoringPose {
            public final int parentTagId;
            public final String id;
            public ScoringPose(int parentTagId, String id) {
                this.parentTagId = parentTagId;
                this.id = id;
            }
        }
        
        // TODO: Update these values with your robot's measurements.
        public static final Transform3d LEFT_ROBOT_TO_CAMERA = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.00, 0.0));
        public static final Transform3d RIGHT_ROBOT_TO_CAMERA = new Transform3d(new Translation3d(0.029, -0.05, 0.089), new Rotation3d(0.0, -0.27, 0.0));
        
        // TODO: This defines the orientation of the ROBOT'S FRONT when scoring.
        // O degrees is field-relative forward (away from your driver station).
        // 180 degrees is field-relative backward (towards your driver station).
        // Update this once you know which side of your robot has the coral mechanism.
        public static final Rotation2d SCORING_ROTATION = Rotation2d.fromDegrees(180.0);

        // TODO: This defines the orientation of the ROBOT'S FRONT when collecting from the wall.
        // Update this once you know which side of your robot has the collector.
        public static final Rotation2d COLLECTING_ROTATION = Rotation2d.fromDegrees(0.0);

        // TODO: Measure this value on the field. This is the distance from the center of an AprilTag to the center of a scoring peg.
        private static final double PEG_OFFSET_METERS = Units.inchesToMeters(12.0);

        private static final List<Integer> CORAL_SCORING_TAG_IDS = List.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
        
        /**
         * A list of all possible scoring locations, without pre-calculated poses.
         * The pose is determined dynamically based on the current alliance.
         */
        public static final List<ScoringPose> ALL_SCORING_POSES = CORAL_SCORING_TAG_IDS.stream()
            .map(tagId -> new ScoringPose(tagId, String.valueOf(tagId)))
            .collect(Collectors.toList());

        /**
         * Calculates the desired field-relative robot pose to align with the closer of two pipes.
         * @param scoringPoseInfo The semantic information about the target tag.
         * @param alliance The current alliance color.
         * @param mode The alignment mode (SCORING or COLLECTING), which determines robot orientation.
         * @param currentRobotPose The robot's current pose, used to determine the closer pipe.
         * @return An Optional containing the calculated Pose2d, or empty if the tag ID is invalid.
         */
        public static Optional<Pose2d> getFieldRelativePose(ScoringPose scoringPoseInfo, Alliance alliance, AlignMode mode, Pose2d currentRobotPose) {
            Pose3d tagPose3d = FieldConstants.APRIL_TAG_FIELD_LAYOUT.get(scoringPoseInfo.parentTagId);
            if (tagPose3d == null) return Optional.empty();

            // 1. Determine the final robot orientation based on the alignment mode.
            Rotation2d targetRobotRotation;
            if (mode == AlignMode.SCORING) {
                targetRobotRotation = (alliance == Alliance.Red) ? SCORING_ROTATION.plus(Rotation2d.fromDegrees(180)) : SCORING_ROTATION;
            } else { // COLLECTING
                targetRobotRotation = (alliance == Alliance.Red) ? COLLECTING_ROTATION.plus(Rotation2d.fromDegrees(180)) : COLLECTING_ROTATION;
            }
            
            // Start with the tag's position
            Translation2d tagTranslation = tagPose3d.toPose2d().getTranslation();
            
            // 2. Calculate the point directly in front of the tag
            Translation2d forwardOffset = (alliance == Alliance.Red)
                ? new Translation2d(-AutoAlignConstants.DESIRED_DISTANCE_METERS, 0)
                : new Translation2d(AutoAlignConstants.DESIRED_DISTANCE_METERS, 0);
            Translation2d stopPoint = tagTranslation.plus(forwardOffset);
    
            // 3. Calculate poses for BOTH left and right pipes
            Rotation2d lateralDirection = (alliance == Alliance.Red) ? Rotation2d.fromDegrees(-90) : Rotation2d.fromDegrees(90);
            
            Translation2d leftLateralOffset = new Translation2d(PEG_OFFSET_METERS, 0).rotateBy(lateralDirection);
            Translation2d rightLateralOffset = new Translation2d(-PEG_OFFSET_METERS, 0).rotateBy(lateralDirection);

            Pose2d leftPose = new Pose2d(stopPoint.plus(leftLateralOffset), targetRobotRotation);
            Pose2d rightPose = new Pose2d(stopPoint.plus(rightLateralOffset), targetRobotRotation);

            // 4. Determine which pose is closer to the robot's current position
            double distToLeft = currentRobotPose.getTranslation().getDistance(leftPose.getTranslation());
            double distToRight = currentRobotPose.getTranslation().getDistance(rightPose.getTranslation());

            // 5. Return the closer pose
            return Optional.of(distToLeft < distToRight ? leftPose : rightPose);
        }
    }

    // TODO: Tune these tolerances for the drive to pose command.
    public static final class AutoConstants {
        public static final double kP_DRIVE_TO_POSE = 1.2;
        public static final double kP_TURN_TO_POSE = 0.05;
        public static final double DRIVE_TO_POSE_TOLERANCE_METERS = 0.6;
        public static final double TURN_TO_POSE_TOLERANCE_DEGREES = 5.0;

        public static final List<Integer> RED_CORAL_COLLECTION_TAG_IDS = List.of(1, 2);
        public static final List<Integer> BLUE_CORAL_COLLECTION_TAG_IDS = List.of(12, 13);
        
        // TODO: Tune these values for simple auto 
        public static final double AUTO_DRIVE_SPEED = 0.25;
        public static final double AUTO_DRIVE_SECONDS = 3.0;
        public static final double AUTO_INTAKE_SPEED = 0.2;
    }
    public static final class FieldConstants {
        public static final double FIELD_LENGTH_METERS = 16.51;
        public static final double FIELD_WIDTH_METERS = 8.2296;

        // Corrected AprilTag poses for the 2025 REEFSCAPE field.
        public static final Map<Integer, Pose3d> APRIL_TAG_FIELD_LAYOUT = Map.ofEntries(
            Map.entry(1, new Pose3d(15.5138, 1.0716, 0.4628, new Rotation3d(0, 0, Units.degreesToRadians(120)))),
            Map.entry(2, new Pose3d(16.204, 3.002, 0.6955, new Rotation3d(0, 0, Units.degreesToRadians(180)))),
            Map.entry(3, new Pose3d(16.204, 4.983, 0.6955, new Rotation3d(0, 0, Units.degreesToRadians(180)))),
            Map.entry(4, new Pose3d(15.5138, 6.9144, 0.4628, new Rotation3d(0, 0, Units.degreesToRadians(240)))),
            Map.entry(5, new Pose3d(0.9964, 1.0716, 0.4628, new Rotation3d(0, 0, Units.degreesToRadians(60)))),
            Map.entry(6, new Pose3d(0.306, 3.002, 0.6955, new Rotation3d(0, 0, 0))),
            Map.entry(7, new Pose3d(0.306, 4.983, 0.6955, new Rotation3d(0, 0, 0))),
            Map.entry(8, new Pose3d(0.9964, 6.9144, 0.4628, new Rotation3d(0, 0, Units.degreesToRadians(300)))),
            Map.entry(9, new Pose3d(13.5905, 7.4994, 0.4636, new Rotation3d(0, 0, Units.degreesToRadians(180)))),
            Map.entry(10, new Pose3d(14.6827, 5.5055, 0.4636, new Rotation3d(0, 0, Units.degreesToRadians(180)))),
            Map.entry(11, new Pose3d(13.5905, 3.5116, 0.4636, new Rotation3d(0, 0, Units.degreesToRadians(180)))),
            Map.entry(12, new Pose3d(16.1671, 3.5116, 0.6096, new Rotation3d(0, 0, Units.degreesToRadians(60)))),
            Map.entry(13, new Pose3d(2.9195, 7.4994, 0.4636, new Rotation3d(0, 0, 0))),
            Map.entry(14, new Pose3d(1.8273, 5.5055, 0.4636, new Rotation3d(0, 0, 0))),
            Map.entry(15, new Pose3d(2.9195, 3.5116, 0.4636, new Rotation3d(0, 0, 0))),
            Map.entry(16, new Pose3d(0.3429, 3.5116, 0.6096, new Rotation3d(0, 0, Units.degreesToRadians(120)))),
            Map.entry(17, new Pose3d(10.2934, 3.9942, 0.8733, new Rotation3d(0, 0, 0))),
            Map.entry(18, new Pose3d(8.3058, 7.728, 0.6096, new Rotation3d(0, 0, Units.degreesToRadians(270)))),
            Map.entry(19, new Pose3d(8.3058, 0.2604, 0.6096, new Rotation3d(0, 0, Units.degreesToRadians(90)))),
            Map.entry(20, new Pose3d(6.2167, 3.9942, 0.8733, new Rotation3d(0, 0, Units.degreesToRadians(180)))),
            Map.entry(21, new Pose3d(8.2042, 7.728, 0.6096, new Rotation3d(0, 0, Units.degreesToRadians(270)))),
            Map.entry(22, new Pose3d(8.2042, 0.2604, 0.6096, new Rotation3d(0, 0, Units.degreesToRadians(90))))
        );
    }
}

