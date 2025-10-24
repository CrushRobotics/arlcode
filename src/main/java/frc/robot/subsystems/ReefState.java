package frc.robot.subsystems;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A helper class to keep track of which specific scoring pegs have been scored on.
 * Updated to be a SubsystemBase so it can be a requirement for commands.
 */
public class ReefState extends SubsystemBase {

    // We store a unique string for each peg, e.g., "A", "B"
    private final Set<String> scoredPegIds = new HashSet<>();
    private String lastTargetedPipeId = null; // Stores the ID of the last pipe targeted by AutoAlign

    /**
     * Marks a specific scoring peg as having been scored on.
     * @param pegId The unique identifier of the peg (e.g., "A", "B").
     */
    public void markScored(String pegId) {
        if (pegId != null && !pegId.isEmpty()) {
            scoredPegIds.add(pegId);
            System.out.println("[ReefState] Marked " + pegId + " as scored.");
        } else {
            System.err.println("[ReefState] Attempted to mark a null or empty pegId as scored.");
        }
    }

    /**
     * Checks if a specific scoring peg has already been scored on.
     * @param pegId The unique identifier of the peg to check.
     * @return True if the peg has been marked as scored, false otherwise.
     */
    public boolean isScored(String pegId) {
        return pegId != null && scoredPegIds.contains(pegId);
    }

    /**
     * Clears all tracked scores and the last targeted pipe ID. Useful for the start of a match.
     */
    public void clear() {
        scoredPegIds.clear();
        lastTargetedPipeId = null;
        System.out.println("[ReefState] Cleared all scores and last target.");
    }

    /**
     * Stores the ID of the last pipe that was targeted by an alignment command.
     * Called by AutoAlignCommand when it locks onto a target.
     * @param pipeId The unique string ID of the pipe (e.g., "A", "B"), or null to clear.
     */
    public void setLastTargetedPipe(String pipeId) {
        this.lastTargetedPipeId = pipeId;
        System.out.println("[ReefState] Last targeted pipe set to: " + (pipeId == null ? "None" : pipeId));
    }

    /**
     * Retrieves the ID of the last pipe that was targeted by an alignment command.
     * Used by the "mark scored" command/button.
     * @return The string ID of the last targeted pipe, or null if none is currently targeted.
     */
    public String getLastTargetedPipe() {
        return this.lastTargetedPipeId;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Can be used for logging scored pegs to SmartDashboard if needed
        // SmartDashboard.putString("ReefState/LastTarget", lastTargetedPipeId != null ? lastTargetedPipeId : "None");
        // SmartDashboard.putString("ReefState/Scored", String.join(", ", scoredPegIds));
    }
}
