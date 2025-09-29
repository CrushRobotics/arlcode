package frc.robot.subsystems;

import java.util.HashSet;
import java.util.Set;

/**
 * A simple helper class to keep track of which AprilTags (representing scoring pipes)
 * have already been scored on.
 */
public class ReefState {

    private final Set<Integer> scoredTagIds = new HashSet<>();

    /**
     * Marks a specific AprilTag ID as having been scored on.
     * @param tagId The ID of the tag to mark.
     */
    public void markScored(int tagId) {
        scoredTagIds.add(tagId);
    }

    /**
     * Checks if a specific AprilTag ID has already been scored on.
     * @param tagId The ID of the tag to check.
     * @return True if the tag has been marked as scored, false otherwise.
     */
    public boolean isScored(int tagId) {
        return scoredTagIds.contains(tagId);
    }

    /**
     * Clears all tracked scores. Useful for the start of a match.
     */
    public void clear() {
        scoredTagIds.clear();
    }
}

