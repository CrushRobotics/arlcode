package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.VisionConstants.ScoringTargetInfo;

/**
 * A utility class to calculate the "cost" of aligning to a specific scoring target.
 * A lower cost is better. The cost function considers distance, alignment with
 * the robot's current travel direction, and whether the target has already been
 * scored on.
 */
public final class AlignmentCostUtil {

    /**
     * Represents a potential alignment target with its associated cost and calculated poses.
     */
    public static class TargetCost implements Comparable<TargetCost> {
        public final ScoringTargetInfo targetInfo; // Contains ID, approach, and final poses
        public final double cost;

        public TargetCost(ScoringTargetInfo targetInfo, double cost) {
            this.targetInfo = targetInfo;
            this.cost = cost;
        }

        @Override
        public int compareTo(TargetCost other) {
            // Sort by cost, lower is better
            return Double.compare(this.cost, other.cost);
        }
    }

    /**
     * Calculates the total cost for aligning to a specific scoring target.
     *
     * @param targetInfo    The ScoringTargetInfo containing ID, approach, and final poses.
     * @param currentPose   The current pose of the robot.
     * @param robotVelocity The current forward velocity of the robot in m/s.
     * @param reefState     The current state of scored targets.
     * @return A TargetCost object containing the original info and the total calculated cost.
     */
    public static TargetCost calculateCost(ScoringTargetInfo targetInfo, Pose2d currentPose, double robotVelocity, ReefState reefState) {

        // Use the final pose for cost calculations
        Pose2d finalTargetPose = targetInfo.finalPose;

        double distanceCost = calculateDistanceCost(currentPose, finalTargetPose);
        double driveCost = calculateDriveDirectionCost(currentPose, finalTargetPose, robotVelocity);
        double reefCost = calculateReefStateCost(targetInfo.id, reefState); // Use the unique pipe ID

        // Apply weights from Constants
        double totalCost = (distanceCost * AutoAlignConstants.DISTANCE_WEIGHT) +
                           (driveCost * AutoAlignConstants.DRIVE_DIRECTION_WEIGHT) +
                           (reefCost * AutoAlignConstants.REEF_STATE_WEIGHT);

        return new TargetCost(targetInfo, totalCost);
    }

    /**
     * Calculates the cost based on the Euclidean distance between the robot and the target.
     * @param currentPose The robot's current pose.
     * @param targetPose The target pose (usually the final stop pose).
     * @return The distance in meters.
     */
    private static double calculateDistanceCost(Pose2d currentPose, Pose2d targetPose) {
        return currentPose.getTranslation().getDistance(targetPose.getTranslation());
    }

    /**
     * Calculates the cost based on how much the robot needs to turn to face the target,
     * considering its current direction of travel. A lower cost means the target is more
     * aligned with the robot's current path.
     * @param currentPose The robot's current pose.
     * @param targetPose The target pose (usually the final stop pose).
     * @param robotVelocity The robot's current forward velocity (m/s).
     * @return A cost value between 0 (perfectly aligned) and 1 (180 degrees off).
     */
    private static double calculateDriveDirectionCost(Pose2d currentPose, Pose2d targetPose, double robotVelocity) {
        // If the robot is barely moving, this cost component is negligible.
        if (Math.abs(robotVelocity) < 0.1) {
            return 0;
        }

        // Vector from robot to target
        Translation2d robotToTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
        // Angle from field X-axis to the target
        Rotation2d angleToTarget = robotToTarget.getAngle();

        // Robot's current direction of travel (heading)
        Rotation2d driveDirection = currentPose.getRotation();

        // If moving backwards, the effective direction is 180 degrees opposite
        if (robotVelocity < 0) {
            driveDirection = driveDirection.plus(Rotation2d.fromDegrees(180));
        }

        // Calculate the absolute difference between the direction of travel and the direction to the target.
        // The difference is normalized to be between 0 and 180 degrees.
        double angleDifferenceDegrees = Math.abs(driveDirection.minus(angleToTarget).getDegrees());

        // Normalize the cost: 0 degrees difference = 0 cost, 180 degrees difference = 1 cost.
        return angleDifferenceDegrees / 180.0;
    }

    /**
     * Calculates the cost based on whether the target peg has already been scored on.
     * @param uniquePipeId The specific ID of the pipe (e.g., "7_LEFT").
     * @param reefState The current state of scored pegs.
     * @return 1.0 if the peg has been scored, 0.0 otherwise.
     */
    private static double calculateReefStateCost(String uniquePipeId, ReefState reefState) {
        // Return a high cost (1.0) if already scored, zero cost otherwise.
        return reefState.isScored(uniquePipeId) ? 1.0 : 0.0;
    }
}
