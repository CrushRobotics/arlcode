package frc.robot.subsystems;

import java.util.HashSet;
import java.util.Set;

/**
 * A helper class to keep track of which specific scoring pegs have been scored on.
 */
public class ReefState {

    // We now store a unique string for each peg, e.g., "7_LEFT"
    private final Set<String> scoredPegIds = new HashSet<>();

    /**
     * Marks a specific scoring peg as having been scored on.
     * @param pegId The unique identifier of the peg (e.g., "7_LEFT").
     */
    public void markScored(String pegId) {
        scoredPegIds.add(pegId);
    }

    /**
     * Checks if a specific scoring peg has already been scored on.
     * @param pegId The unique identifier of the peg to check.
     * @return True if the peg has been marked as scored, false otherwise.
     */
    public boolean isScored(String pegId) {
        return scoredPegIds.contains(pegId);
    }

    /**
     * Clears all tracked scores. Useful for the start of a match.
     */
    public void clear() {
        scoredPegIds.clear();
    }
}
