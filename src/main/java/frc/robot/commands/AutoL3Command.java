package frc.robot.commands;

import java.util.Comparator;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
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
        // Find the closest scoring AprilTag to the coral collection zone
        Pose2d secondTargetPose = findClosestScoringPose(AutoConstants.CORAL_COLLECTION_POSE)
            .orElse(new Pose2d(6.21, 4.0, new edu.wpi.first.math.geometry.Rotation2d())); // Default if none found

        addCommands(
            // 1. Scan and score pre-loaded coral on L3
            new AutoAlignCommand(drive, localization, vision, reefState),
            new SetScoringPositionCommand(arm, elevator, ScoringLevel.L3),
            new CoralIntakeCommand(coralIntake, CoralIntakeDirection.Down).withTimeout(1.0),

            // 2. Go to collect another coral
            new DriveToPoseCommand(drive, localization, AutoConstants.CORAL_COLLECTION_POSE),
            new CoralIntakeCommand(coralIntake, CoralIntakeDirection.Up).withTimeout(2.0),

            // 3. Drive to the closest L3 scoring position and score
            new DriveToPoseCommand(drive, localization, secondTargetPose),
            new AutoAlignCommand(drive, localization, vision, reefState),
            new SetScoringPositionCommand(arm, elevator, ScoringLevel.L3),
            new CoralIntakeCommand(coralIntake, CoralIntakeDirection.Down).withTimeout(1.0)
        );
    }

    /**
     * Finds the closest valid scoring AprilTag to a given position on the field.
     * @param fromPose The pose to measure distance from.
     * @return An Optional containing the Pose2d of the closest tag, or empty if none are valid.
     */
    private Optional<Pose2d> findClosestScoringPose(Pose2d fromPose) {
        return VisionConstants.CORAL_SCORING_TAG_IDS.stream()
            .map(id -> FieldConstants.APRIL_TAG_FIELD_LAYOUT.get(id))
            .filter(pose3d -> pose3d != null)
            .map(pose3d -> pose3d.toPose2d())
            .min(Comparator.comparingDouble(tagPose -> 
                tagPose.getTranslation().getDistance(fromPose.getTranslation())
            ));
    }
}
