package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.ReefState;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.AlignmentCostUtil;
import frc.robot.subsystems.AlignmentCostUtil.TargetCost;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

public class AutoAlignCommand extends Command {

    private final CANDriveSubsystem driveSubsystem;
    private final LocalizationSubsystem localizationSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final ReefState reefState;

    private final PIDController turnController;
    private final PIDController driveController;

    private Optional<TargetCost> bestTarget = Optional.empty();

    public AutoAlignCommand(CANDriveSubsystem drive, LocalizationSubsystem localization, VisionSubsystem vision, ReefState reef) {
        this.driveSubsystem = drive;
        this.localizationSubsystem = localization;
        this.visionSubsystem = vision;
        this.reefState = reef;

        turnController = new PIDController(AutoAlignConstants.kP_TURN, AutoAlignConstants.kI_TURN, AutoAlignConstants.kD_TURN);
        turnController.enableContinuousInput(-180, 180); // Angles are circular
        turnController.setTolerance(AutoAlignConstants.TURN_TOLERANCE_DEGREES);

        driveController = new PIDController(AutoAlignConstants.kP_DRIVE, AutoAlignConstants.kI_DRIVE, AutoAlignConstants.kD_DRIVE);
        driveController.setTolerance(AutoAlignConstants.DRIVE_TOLERANCE_METERS);
        
        addRequirements(drive, localization); // Vision is read-only, no need to require
    }

    @Override
    public void initialize() {
        bestTarget = findBestTarget();
        if (bestTarget.isPresent()) {
            System.out.println("Auto Align Initialized: Targeting Tag ID " + bestTarget.get().tagId);
        } else {
            System.out.println("Auto Align Initialized: No valid targets found.");
        }
    }

    @Override
    public void execute() {
        if (bestTarget.isEmpty()) {
            driveSubsystem.drive(0, 0);
            return;
        }

        Pose2d currentPose = localizationSubsystem.getPose();
        Pose2d targetPose = bestTarget.get().targetPose;
        
        // --- Rotation Control ---
        // We want to face the target.
        Translation2d translationToTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
        Rotation2d desiredRotation = translationToTarget.getAngle();
        
        double rotationSpeed = turnController.calculate(
            currentPose.getRotation().getDegrees(),
            desiredRotation.getDegrees()
        );
        
        // --- Forward/Backward Control ---
        // We want to be a certain distance away from the target.
        double currentDistance = translationToTarget.getNorm();
        
        double driveSpeed = driveController.calculate(currentDistance, AutoAlignConstants.DESIRED_DISTANCE_METERS);

        // Apply a max speed
        driveSpeed = Math.max(-0.5, Math.min(0.5, driveSpeed));
        rotationSpeed = Math.max(-0.5, Math.min(0.5, rotationSpeed));

        driveSubsystem.drive(driveSpeed, rotationSpeed);

        SmartDashboard.putNumber("AutoAlign/TargetID", bestTarget.get().tagId);
        SmartDashboard.putNumber("AutoAlign/DistanceError", currentDistance - AutoAlignConstants.DESIRED_DISTANCE_METERS);
        SmartDashboard.putNumber("AutoAlign/RotationError", currentPose.getRotation().getDegrees() - desiredRotation.getDegrees());
    }

    private Optional<TargetCost> findBestTarget() {
        List<TargetCost> potentialTargets = new ArrayList<>();
        Pose2d currentPose = localizationSubsystem.getPose();
        double velocity = driveSubsystem.getForwardVelocityMetersPerSec();

        for (int tagId : VisionConstants.CORAL_SCORING_TAG_IDS) {
            AlignmentCostUtil.calculateCost(tagId, currentPose, velocity, reefState)
                .ifPresent(potentialTargets::add);
        }

        if (potentialTargets.isEmpty()) {
            return Optional.empty();
        }

        // The best target is the one with the lowest cost
        Collections.sort(potentialTargets);
        return Optional.of(potentialTargets.get(0));
    }

    @Override
    public boolean isFinished() {
        // The command is finished if we are at both the distance and angle setpoints.
        return driveController.atSetpoint() && turnController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0);
    }

    /**
     * A helper method to create an InstantCommand that marks the current best target as scored.
     * @return A command that can be bound to a button.
     */
    public Command getMarkScoredCommand() {
        // ReefState is not a subsystem, so it should not be passed as a requirement.
        return new InstantCommand(() -> {
            Optional<TargetCost> target = findBestTarget();
            target.ifPresent(t -> {
                reefState.markScored(t.tagId);
                System.out.println("Marked Tag " + t.tagId + " as scored.");
            });
        });
    }
}

