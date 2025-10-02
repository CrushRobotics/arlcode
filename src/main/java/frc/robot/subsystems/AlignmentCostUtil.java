package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.VisionConstants.ScoringPose;
import java.util.Optional;

/**
 * A utility class to calculate the "cost" of aligning to a specific scoring pose.
 * A lower cost is better. The cost function considers distance, alignment with
 * the robot's current travel direction, and whether the target has already been
 * scored on.
 */
public final class AlignmentCostUtil {

    /**
     * Represents a potential alignment target with its associated cost.
     */
    public static class TargetCost implements Comparable<TargetCost> {
        public final ScoringPose scoringPose;
        public final double cost;

        public TargetCost(ScoringPose scoringPose, double cost) {
            this.scoringPose = scoringPose;
            this.cost = cost;
        }

        @Override
        public int compareTo(TargetCost other) {
            return Double.compare(this.cost, other.cost);
        }
    }

    /**
     * Calculates the total cost for aligning to a specific scoring pose.
     *
     * @param scoringPose   The scoring pose to evaluate.
     * @param currentPose   The current pose of the robot.
     * @param robotVelocity The current forward velocity of the robot in m/s.
     * @param reefState     The current state of scored targets.
     * @return An Optional containing the TargetCost.
     */
    public static Optional<TargetCost> calculateCost(ScoringPose scoringPose, Pose2d currentPose, double robotVelocity, ReefState reefState) {
        
        Pose2d targetPose = scoringPose.pose;

        double distanceCost = calculateDistanceCost(currentPose, targetPose);
        double driveCost = calculateDriveDirectionCost(currentPose, targetPose, robotVelocity);
        double reefCost = calculateReefStateCost(scoringPose.id, reefState);

        double totalCost = (distanceCost * AutoAlignConstants.DISTANCE_WEIGHT) + 
                           (driveCost * AutoAlignConstants.DRIVE_DIRECTION_WEIGHT) + 
                           (reefCost * AutoAlignConstants.REEF_STATE_WEIGHT);

        return Optional.of(new TargetCost(scoringPose, totalCost));
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

    private static double calculateReefStateCost(String scoringPoseId, ReefState reefState) {
        // If the target has been scored, return a high cost. Otherwise, zero cost.
        return reefState.isScored(scoringPoseId) ? 1.0 : 0.0;
    }
}
