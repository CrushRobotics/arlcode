package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.PipeBase; // Import PipeBase
import frc.robot.Constants.VisionConstants.ScoringTargetInfo;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.ReefState;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.AlignmentCostUtil;
import frc.robot.subsystems.AlignmentCostUtil.TargetCost;

import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Function; // Added for explicit cast
import java.util.stream.Collectors;

/**
 * AutoAlignCommand rewritten using fixed pipe poses selected based on distance and AprilTag visibility confirmation.
 * 1. Checks if any relevant scoring AprilTags are visible.
 * 2. Finds the closest *fixed* pipe pose (from Constants.VisionConstants.PIPES) to the robot.
 * 3. Calculates approach and final poses relative to this fixed pipe pose.
 * 4. Generates two trajectories: current -> approach and approach -> final.
 * 5. Executes the trajectories sequentially.
 */
public class AutoAlignCommand extends SequentialCommandGroup {

    /**
     * Defines the purpose of the alignment, which determines the robot's final orientation.
     */
    public enum AlignMode {
        SCORING,
        COLLECTING // Note: Pipe poses are likely only for SCORING, collecting might need different logic
    }

    private final ReefState reefState; // Keep reefState for marking scored

    // Store target info for marking scored later
    private static ScoringTargetInfo lockedTargetInfo = null;

    /**
     * Creates a new AutoAlignCommand sequence using fixed pipe poses.
     * Defers construction until scheduled to get current robot pose and alliance.
     *
     * @param drive         The drive subsystem.
     * @param localization  The localization subsystem.
     * @param vision        The vision subsystem (used for visibility check).
     * @param reef          The reef state tracker.
     * @param mode          The alignment mode (SCORING recommended).
     */
    public AutoAlignCommand(CANDriveSubsystem drive, LocalizationSubsystem localization, VisionSubsystem vision, ReefState reef, AlignMode mode) {
        this.reefState = reef;
        addRequirements(drive, localization, vision); // Add subsystems as requirements

        // Use Commands.defer() to delay the creation of the sequence until the command starts
        addCommands(Commands.defer(() -> {
            Optional<Alliance> allianceOpt = DriverStation.getAlliance();
            if (allianceOpt.isEmpty()) {
                System.err.println("AutoAlignCommand: Alliance not set!");
                return Commands.none(); // Return an empty command if no alliance
            }
            Alliance alliance = allianceOpt.get();
            Pose2d currentPose = localization.getPose();

            // Find the best target based on fixed poses and tag visibility
            Optional<TargetCost> bestTargetOpt = findBestTargetFromFixedPipes(vision, alliance, mode, currentPose, reef);

            if (bestTargetOpt.isPresent()) {
                TargetCost bestTarget = bestTargetOpt.get();
                if (bestTarget.targetInfo == null) {
                    System.err.println("AutoAlignCommand: Best target found but targetInfo is null!");
                    lockedTargetInfo = null;
                    return Commands.none();
                }

                lockedTargetInfo = bestTarget.targetInfo; // Store the chosen target info

                SmartDashboard.putString("AutoAlign/TargetID", lockedTargetInfo.id); // Uses PipeBase name now
                SmartDashboard.putString("AutoAlign/TargetApproachPose", lockedTargetInfo.approachPose.toString());
                SmartDashboard.putString("AutoAlign/TargetFinalPose", lockedTargetInfo.finalPose.toString());
                System.out.println("Auto Align Target Locked: ID=" + lockedTargetInfo.id + ", Final=" + lockedTargetInfo.finalPose);

                // Create the sequence of trajectory commands
                return new SequentialCommandGroup(
                    new DriveToPoseTrajectoryCommand(drive, localization, currentPose, lockedTargetInfo.approachPose),
                    new DriveToPoseTrajectoryCommand(drive, localization, lockedTargetInfo.approachPose, lockedTargetInfo.finalPose)
                ).finallyDo(interrupted -> { // Handle interruption
                    if (interrupted) {
                         System.out.println("AutoAlignCommand Sequence Interrupted. Target lock cleared.");
                         lockedTargetInfo = null; // Clear lock if interrupted
                     } else {
                         System.out.println("AutoAlignCommand Sequence Finished normally.");
                         // Keep lockedTargetInfo for potential marking scored
                     }
                 });
            } else {
                System.err.println("AutoAlignCommand: No suitable target found (no scoring tags visible or error calculating poses)!");
                lockedTargetInfo = null; // Clear locked target
                SmartDashboard.putString("AutoAlign/TargetID", "None");
                return Commands.none(); // Return an empty command if no target found
            }
        }, Set.of(drive, localization, vision))); // Declare dependencies for defer
    }

     /**
      * Finds the best scoring/collection target using the fixed pipe poses from Constants,
      * selecting the closest one only if relevant AprilTags are visible.
      *
      * @param vision        The vision subsystem.
      * @param alliance      The current alliance.
      * @param mode          The alignment mode.
      * @param currentPose   The robot's current pose.
      * @param reefState     The state of scored pegs.
      * @return An Optional containing the best TargetCost, or empty if none are suitable.
      */
    private Optional<TargetCost> findBestTargetFromFixedPipes(VisionSubsystem vision, Alliance alliance, AlignMode mode, Pose2d currentPose, ReefState reefState) {
        Set<Integer> visibleTagIds = vision.getVisibleTagIds();
        double currentVelocity = 0; // Assuming starting from stop

        // 1. Check if ANY valid scoring tag is visible as a prerequisite
        boolean scoringZoneVisible = false;
        for(int tagId : VisionConstants.VALID_SCORING_TAG_IDS) {
            if (visibleTagIds.contains(tagId)) {
                scoringZoneVisible = true;
                break;
            }
        }

        if (!scoringZoneVisible) {
            System.out.println("AutoAlign/findBestTarget: No VALID_SCORING_TAG_IDS visible. Cannot select fixed target.");
            SmartDashboard.putString("AutoAlign/VisibleTags", visibleTagIds.stream().map(String::valueOf).collect(Collectors.joining(", ")));
            return Optional.empty();
        }
         SmartDashboard.putString("AutoAlign/VisibleTags", visibleTagIds.stream().map(String::valueOf).collect(Collectors.joining(", ")));


        // 2. Get all potential target infos (approach/final poses) based on the FIXED PIPES list
        List<ScoringTargetInfo> potentialTargetInfos = VisionConstants.getAllPotentialTargets(alliance, mode);

        if (potentialTargetInfos.isEmpty()) {
             System.out.println("AutoAlign/findBestTarget: No potential target poses generated from fixed PIPES list.");
            return Optional.empty();
        }

        // 3. Calculate the cost for each potential fixed target
        List<TargetCost> potentialTargetsWithCost = potentialTargetInfos.stream()
            // Explicitly cast lambda to resolve potential inference issues
            .map((Function<ScoringTargetInfo, TargetCost>) targetInfo -> AlignmentCostUtil.calculateCost(targetInfo, currentPose, currentVelocity, reefState))
            .collect(Collectors.toList());


        // 4. Sort targets by cost (lowest cost is best - primarily distance now)
        Collections.sort(potentialTargetsWithCost);

        // Debugging: Print costs
        // System.out.println("--- Target Costs (Fixed Pipes) ---");
        // potentialTargetsWithCost.forEach(tc -> System.out.printf("ID: %s, Cost: %.2f\n", tc.targetInfo.id, tc.cost));
        // System.out.println("----------------------------------");


        // 5. Return the lowest cost target
        if (potentialTargetsWithCost.isEmpty()) {
             System.out.println("AutoAlign/findBestTarget: Cost calculation resulted in empty list.");
             return Optional.empty();
        }
        return Optional.of(potentialTargetsWithCost.get(0)); // Return the closest valid pipe
    }


    /**
     * Gets an InstantCommand that marks the currently locked target as scored in the ReefState.
     * Should be triggered after successfully scoring. Uses the PipeBase name as the ID.
     * @return The command to mark the target as scored.
     */
    public Command getMarkScoredCommand() {
        // Removed reefState as requirement since it's not a Subsystem
        return new InstantCommand(() -> {
            if (lockedTargetInfo != null) {
                reefState.markScored(lockedTargetInfo.id); // Mark using the PipeBase name (e.g., "A", "B")
                System.out.println("Marked " + lockedTargetInfo.id + " as scored.");
                // Optionally clear the lock after marking
                // lockedTargetInfo = null;
            } else {
                 System.out.println("Mark Scored: No target was locked.");
            }
        }); // Removed reefState requirement
    }

     // Removed the end() override as it's final in SequentialCommandGroup
     // Cleanup logic moved to finallyDo() in the constructor's defer block.

}

