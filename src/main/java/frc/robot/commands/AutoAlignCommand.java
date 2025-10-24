package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer; // Added for periodic checking
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants; // Added import
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.PipeBase; // Import PipeBase
import frc.robot.Constants.VisionConstants.ScoringTargetInfo;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.ReefState;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.AlignmentCostUtil;
import frc.robot.subsystems.AlignmentCostUtil.TargetCost;
import frc.robot.LimelightHelpers; // Added import for validPoseEstimate if needed elsewhere, kept for safety

import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Function; // Added for explicit cast
import java.util.stream.Collectors;

/**
 * AutoAlignCommand: Finds the best scoring target pipe and drives to it using trajectories.
 * 1. Defers target selection until command start.
 * 2. Periodically re-evaluates the best target during the approach phase.
 * 3. Finds the closest *fixed* pipe pose (from Constants.VisionConstants.PIPES) that is visible and not scored.
 * 4. Calculates approach and final poses relative to this fixed pipe pose.
 * 5. Generates trajectories: current -> approach, approach -> final.
 * 6. Executes trajectories sequentially, updating target if a better one appears during approach.
 */
public class AutoAlignCommand extends Command { // Changed from SequentialCommandGroup

    private final CANDriveSubsystem driveSubsystem;
    private final LocalizationSubsystem localizationSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final ReefState reefState;
    private final AlignMode alignMode;

    private Command currentTrajectoryCommand = null;
    private ScoringTargetInfo currentTargetInfo = null;
    private boolean isApproaching = false;
    private Timer checkTimer = new Timer();
    private static final double CHECK_INTERVAL_SECONDS = 0.5; // How often to re-evaluate target

    /**
     * Defines the purpose of the alignment, which determines the robot's final orientation.
     */
    public enum AlignMode {
        SCORING,
        COLLECTING // Note: Pipe poses are likely only for SCORING
    }

    /**
     * Creates a new AutoAlignCommand.
     *
     * @param drive         The drive subsystem.
     * @param localization  The localization subsystem.
     * @param vision        The vision subsystem (used for visibility check).
     * @param reef          The reef state tracker.
     * @param mode          The alignment mode (SCORING recommended).
     */
    public AutoAlignCommand(CANDriveSubsystem drive, LocalizationSubsystem localization, VisionSubsystem vision, ReefState reef, AlignMode mode) {
        this.driveSubsystem = drive;
        this.localizationSubsystem = localization;
        this.visionSubsystem = vision;
        this.reefState = reef;
        this.alignMode = mode;

        addRequirements(drive, localization, vision, reefState); // Add subsystems as requirements
    }

    @Override
    public void initialize() {
        System.out.println("AutoAlignCommand Initializing...");
        currentTargetInfo = null; // Reset target
        currentTrajectoryCommand = null;
        isApproaching = false;
        checkTimer.reset();
        checkTimer.start();

        // Initial target selection
        selectAndStartTarget();
    }

    private void selectAndStartTarget() {
        Optional<Alliance> allianceOpt = DriverStation.getAlliance();
        if (allianceOpt.isEmpty()) {
            System.err.println("AutoAlignCommand: Alliance not set!");
            this.cancel(); // Cancel if no alliance
            return;
        }
        Alliance alliance = allianceOpt.get();
        Pose2d currentPose = localizationSubsystem.getPose();

        Optional<TargetCost> bestTargetOpt = findBestTargetFromFixedPipes(currentPose, alliance);

        if (bestTargetOpt.isPresent()) {
            TargetCost bestTarget = bestTargetOpt.get();
            ScoringTargetInfo newTargetInfo = bestTarget.targetInfo;

            // Only switch if it's a new target or the first target
            if (currentTargetInfo == null || !currentTargetInfo.id.equals(newTargetInfo.id)) {
                System.out.println("AutoAlignCommand: Selecting Target ID=" + newTargetInfo.id);
                currentTargetInfo = newTargetInfo;
                reefState.setLastTargetedPipe(currentTargetInfo.id); // Update ReefState

                SmartDashboard.putString("AutoAlign/TargetID", currentTargetInfo.id);
                SmartDashboard.putString("AutoAlign/TargetApproachPose", currentTargetInfo.approachPose.toString());
                SmartDashboard.putString("AutoAlign/TargetFinalPose", currentTargetInfo.finalPose.toString());

                // Cancel any existing trajectory
                if (currentTrajectoryCommand != null && !currentTrajectoryCommand.isFinished()) {
                    currentTrajectoryCommand.cancel();
                }

                // Start the new sequence
                currentTrajectoryCommand = new SequentialCommandGroup(
                    new DriveToPoseTrajectoryCommand(driveSubsystem, localizationSubsystem, currentPose, currentTargetInfo.approachPose)
                        .beforeStarting(() -> isApproaching = true) // Mark start of approach
                        .andThen(() -> isApproaching = false), // Mark end of approach
                    new DriveToPoseTrajectoryCommand(driveSubsystem, localizationSubsystem, currentTargetInfo.approachPose, currentTargetInfo.finalPose)
                );
                currentTrajectoryCommand.schedule();

            } else {
                 System.out.println("AutoAlignCommand: Keeping current target ID=" + currentTargetInfo.id);
            }

        } else {
            System.err.println("AutoAlignCommand: No suitable target found!");
            SmartDashboard.putString("AutoAlign/TargetID", "None");
            // If already driving, let it finish? Or cancel? For now, let it finish the current segment.
            // If no target was ever found, cancel the command.
            if (currentTargetInfo == null) {
                reefState.setLastTargetedPipe(null); // Clear target in ReefState
                this.cancel();
            }
        }
    }


    @Override
    public void execute() {
        // Periodically check for a better target ONLY during the approach phase
        if (isApproaching && checkTimer.hasElapsed(CHECK_INTERVAL_SECONDS)) {
            System.out.println("AutoAlignCommand: Re-evaluating target...");
            selectAndStartTarget(); // Re-select target, may interrupt and restart trajectory
            checkTimer.reset(); // Reset timer for next check
        }
    }


    @Override
    public boolean isFinished() {
        // Finished when the scheduled trajectory command sequence is done
        return currentTrajectoryCommand != null && currentTrajectoryCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("AutoAlignCommand ended. Interrupted: " + interrupted);
        // Ensure the trajectory is cancelled if the command is interrupted externally
        if (interrupted && currentTrajectoryCommand != null) {
            currentTrajectoryCommand.cancel();
        }
        driveSubsystem.stop(); // Ensure robot stops
        if (interrupted) {
            reefState.setLastTargetedPipe(null); // Clear target if interrupted
        }
        checkTimer.stop();
        // currentTargetInfo and state in ReefState remain if finished normally
    }

    /**
     * Finds the best scoring target based on visibility, score status, and distance.
     */
    private Optional<TargetCost> findBestTargetFromFixedPipes(Pose2d currentPose, Alliance alliance) {
        Set<Integer> visibleTagIds = visionSubsystem.getVisibleTagIds();
        double currentVelocity = driveSubsystem.getChassisSpeeds().vxMetersPerSecond; // Use current speed

        // 1. Check visibility prerequisite
        boolean scoringZoneVisible = VisionConstants.VALID_SCORING_TAG_IDS.stream()
                                            .anyMatch(visibleTagIds::contains);

        if (!scoringZoneVisible) {
            System.out.println("AutoAlign/findBest: No VALID_SCORING_TAG_IDS visible.");
            SmartDashboard.putString("AutoAlign/VisibleTags", visibleTagIds.stream().map(String::valueOf).collect(Collectors.joining(", ")));
            return Optional.empty();
        }
        SmartDashboard.putString("AutoAlign/VisibleTags", visibleTagIds.stream().map(String::valueOf).collect(Collectors.joining(", ")));


        // 2. Get all potential target infos
        List<ScoringTargetInfo> potentialTargetInfos = VisionConstants.getAllPotentialTargets(alliance, alignMode);
        if (potentialTargetInfos.isEmpty()) {
            System.out.println("AutoAlign/findBest: No potential target poses generated.");
            return Optional.empty();
        }

        // 3. Calculate cost for each
        List<TargetCost> potentialTargetsWithCost = potentialTargetInfos.stream()
            .map((Function<ScoringTargetInfo, TargetCost>) targetInfo ->
                 AlignmentCostUtil.calculateCost(targetInfo, currentPose, currentVelocity, reefState))
            .collect(Collectors.toList());

        // 4. Sort by cost (lowest is best)
        Collections.sort(potentialTargetsWithCost);

        // 5. Return the lowest cost target (which implicitly filters out scored targets due to high cost)
        if (potentialTargetsWithCost.isEmpty()) {
             System.out.println("AutoAlign/findBest: Cost calculation resulted in empty list.");
             return Optional.empty();
        }
        // Check if the best cost is excessively high (meaning all likely scored)
        // Fully qualify the constant reference to avoid ambiguity
        if (potentialTargetsWithCost.get(0).cost >= AlignmentCostUtil.calculateReefStateCost("dummy", reefState) * frc.robot.Constants.AutoAlignConstants.REEF_STATE_WEIGHT) {
             System.out.println("AutoAlign/findBest: All targets appear to be scored (lowest cost is high).");
             return Optional.empty();
        }

        return Optional.of(potentialTargetsWithCost.get(0));
    }
}

