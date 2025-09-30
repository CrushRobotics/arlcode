package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.ReefState;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.FieldConstants;
import java.util.Optional;

/**
 * A utility class to calculate the "cost" of aligning to a specific AprilTag.
 * A lower cost is better. The cost function considers distance, alignment with
 * the robot's current travel direction, and whether the target has already been
 * scored on.
 */
public final class AlignmentCostUtil {

    /**
     * Represents a potential alignment target with its associated cost.
     */
    public static class TargetCost implements Comparable<TargetCost> {
        public final int tagId;
        public final double cost;
        public final Pose2d targetPose;

        public TargetCost(int tagId, double cost, Pose2d targetPose) {
            this.tagId = tagId;
            this.cost = cost;
            this.targetPose = targetPose;
        }

        @Override
        public int compareTo(TargetCost other) {
            return Double.compare(this.cost, other.cost);
        }
    }

    /**
     * Calculates the total cost for aligning to a specific AprilTag.
     *
     * @param tagId         The ID of the AprilTag to evaluate.
     * @param currentPose   The current pose of the robot.
     * @param robotVelocity The current forward velocity of the robot in m/s.
     * @param reefState     The current state of scored targets.
     * @return An Optional containing the TargetCost if the tag is valid, otherwise empty.
     */
    public static Optional<TargetCost> calculateCost(int tagId, Pose2d currentPose, double robotVelocity, ReefState reefState) {
        // Get the AprilTag's pose from our field constants
        var tagPose3d = FieldConstants.APRIL_TAG_FIELD_LAYOUT.get(tagId);
        if (tagPose3d == null) {
            return Optional.empty(); // This tag isn't on our map
        }
        Pose2d tagPose = tagPose3d.toPose2d();

        double distanceCost = calculateDistanceCost(currentPose, tagPose);
        double driveCost = calculateDriveDirectionCost(currentPose, tagPose, robotVelocity);
        double reefCost = calculateReefStateCost(tagId, reefState);

        double totalCost = (distanceCost * AutoAlignConstants.DISTANCE_WEIGHT) + 
                           (driveCost * AutoAlignConstants.DRIVE_DIRECTION_WEIGHT) + 
                           (reefCost * AutoAlignConstants.REEF_STATE_WEIGHT);

        return Optional.of(new TargetCost(tagId, totalCost, tagPose));
    }

    private static double calculateDistanceCost(Pose2d currentPose, Pose2d targetPose) {
        return currentPose.getTranslation().getDistance(targetPose.getTranslation());
    }

    private static double calculateDriveDirectionCost(Pose2d currentPose, Pose2d targetPose, double robotVelocity) {
        if (Math.abs(robotVelocity) < 0.1) {
            return 0; // Robot is not moving, so no cost
        }

        Translation2d robotToTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
        Rotation2d driveDirection = currentPose.getRotation();
        
        // If moving backwards, flip the direction
        if (robotVelocity < 0) {
            driveDirection = driveDirection.plus(Rotation2d.fromDegrees(180));
        }

        Rotation2d angleToTarget = robotToTarget.getAngle();
        
        // This gives the difference in angle between where we are going and where the target is.
        // A smaller difference is better.
        double angleDifference = Math.abs(driveDirection.minus(angleToTarget).getDegrees());

        // We normalize it to a 0-1 range where 0 is perfect alignment and 1 is 180 degrees off.
        return angleDifference / 180.0;
    }

    private static double calculateReefStateCost(int tagId, ReefState reefState) {
        // If the target has been scored, return a high cost. Otherwise, zero cost.
        return reefState.isScored(tagId) ? 1.0 : 0.0;
    }
}

