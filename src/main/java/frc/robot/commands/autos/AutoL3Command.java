package frc.robot.commands.autos;

import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.SetScoringPositionCommand;
import frc.robot.commands.CoralIntakeCommand.CoralIntakeDirection;
import frc.robot.commands.SetScoringPositionCommand.ScoringLevel;
import frc.robot.subsystems.CANArmSubsystem;
import frc.robot.subsystems.CANCoralIntakeSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANElevatorSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.ReefState;
import frc.robot.subsystems.VisionSubsystem;

public class AutoL3Command extends SequentialCommandGroup {

public AutoL3Command(
    CANDriveSubsystem drive, 
    LocalizationSubsystem localization, 
    VisionSubsystem vision,
    CANArmSubsystem arm, 
    CANElevatorSubsystem elevator, 
    CANCoralIntakeSubsystem coralIntake,
    ReefState reefState
) {

    addCommands(
        // 1. Scan and score pre-loaded coral on L3
        new AutoAlignCommand(drive, localization, vision, reefState),
        new SetScoringPositionCommand(arm, elevator, ScoringLevel.L3),
        new CoralIntakeCommand(coralIntake, CoralIntakeDirection.Down).withTimeout(1.0),

        // 2. Go to collect another coral
        // Move arm to a safe travel position first
        Commands.runOnce(() -> arm.setPosition(ArmConstants.HOME_POSITION_ROTATIONS)),

        // Defer the creation of the DriveToPoseCommand until this step is reached.
        // This allows us to get the robot's current pose and alliance dynamically.
        Commands.defer(() -> {
            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isEmpty()) {
                return Commands.none(); // No alliance, do nothing
            }

            List<Integer> collectionTagIds = alliance.get() == Alliance.Red 
                ? AutoConstants.RED_CORAL_COLLECTION_TAG_IDS 
                : AutoConstants.BLUE_CORAL_COLLECTION_TAG_IDS;

            Pose2d currentPose = localization.getPose();
            Pose2d collectionPose = findClosestPoseFromIds(currentPose, collectionTagIds)
                .orElse(currentPose); // Default to staying put if no tags found

            return new DriveToPoseCommand(drive, localization, collectionPose);
        }, Set.of(drive, localization, arm)),

        new CoralIntakeCommand(coralIntake, CoralIntakeDirection.Up).withTimeout(2.0),

        // 3. Drive to the closest scoring position and score
        // Move arm to a safe travel position again
        Commands.runOnce(() -> arm.setPosition(ArmConstants.HOME_POSITION_ROTATIONS)),
        
        // The AutoAlignCommand will find the best available scoring pose and drive to it.
        // The redundant DriveToPoseCommand has been removed.
        new AutoAlignCommand(drive, localization, vision, reefState),
        new SetScoringPositionCommand(arm, elevator, ScoringLevel.L3),
        new CoralIntakeCommand(coralIntake, CoralIntakeDirection.Down).withTimeout(1.0)
    );
}

/**
 * Finds the closest valid AprilTag to a given position from a list of IDs.
 * @param fromPose The pose to measure distance from.
 * @param tagIds The list of tag IDs to check.
 * @return An Optional containing the Pose2d of the closest tag, or empty if none are valid.
 */
private Optional<Pose2d> findClosestPoseFromIds(Pose2d fromPose, List<Integer> tagIds) {
    return tagIds.stream()
        .map(id -> FieldConstants.APRIL_TAG_FIELD_LAYOUT.get(id))
        .filter(pose3d -> pose3d != null)
        .map(pose3d -> pose3d.toPose2d())
        .min(Comparator.comparingDouble(tagPose -> 
            tagPose.getTranslation().getDistance(fromPose.getTranslation())
        ));
}
}