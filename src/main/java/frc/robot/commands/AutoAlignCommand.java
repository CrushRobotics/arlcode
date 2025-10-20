package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    private final ReefState reefState;

    private final ProfiledPIDController turnController;
    private final ProfiledPIDController driveController;

    private Optional<TargetCost> bestTarget = Optional.empty();

    public AutoAlignCommand(CANDriveSubsystem drive, LocalizationSubsystem localization, VisionSubsystem vision, ReefState reef) {
        this.driveSubsystem = drive;
        this.localizationSubsystem = localization;
        this.reefState = reef;

        turnController = new ProfiledPIDController(
            AutoAlignConstants.kP_TURN, 
            AutoAlignConstants.kI_TURN, 
            AutoAlignConstants.kD_TURN,
            new TrapezoidProfile.Constraints(360, 360) // Max angular velocity and acceleration in deg/s
        );
        turnController.enableContinuousInput(-180, 180);
        turnController.setTolerance(AutoAlignConstants.TURN_TOLERANCE_DEGREES);

        driveController = new ProfiledPIDController(
            AutoAlignConstants.kP_DRIVE, 
            AutoAlignConstants.kI_DRIVE, 
            AutoAlignConstants.kD_DRIVE,
            new TrapezoidProfile.Constraints(1.0, 1.0) // Max velocity and acceleration in m/s
        );
        driveController.setTolerance(AutoAlignConstants.DRIVE_TOLERANCE_METERS);
        
        addRequirements(drive, localization);
    }

    @Override
    public void initialize() {
        bestTarget = findBestTarget();
        if (bestTarget.isPresent()) {
            System.out.println("Auto Align Initialized: Targeting " + bestTarget.get().scoringPose.id);
            // Reset controllers to current state
            driveController.reset(localizationSubsystem.getPose().getTranslation().getDistance(bestTarget.get().scoringPose.pose.getTranslation()));
            turnController.reset(localizationSubsystem.getPose().getRotation().getDegrees());
        } else {
            System.out.println("Auto Align Initialized: No valid targets found.");
        }
    }

    @Override
    public void execute() {
        if (bestTarget.isEmpty()) {
            driveSubsystem.stop();
            return;
        }

        // If we are at the setpoint, stop moving to prevent oscillation.
        if (driveController.atSetpoint() && turnController.atSetpoint()) {
            driveSubsystem.stop();
            return;
        }

        Pose2d currentPose = localizationSubsystem.getPose();
        Pose2d targetPose = bestTarget.get().scoringPose.pose;
        
        // --- Calculations for both controllers ---
        Translation2d translationToTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
        double currentDistance = translationToTarget.getNorm();
        Rotation2d desiredRotation = translationToTarget.getAngle();

        // --- Rotation Control ---
        double rotationSpeed = turnController.calculate(
            currentPose.getRotation().getDegrees(),
            desiredRotation.getDegrees()
        );

        // --- Translation Control ---
        double driveSpeed = -driveController.calculate(currentDistance, AutoAlignConstants.DESIRED_DISTANCE_METERS);

        // Scale drive speed by how much we're facing the target. This prevents driving sideways.
        double angleError = currentPose.getRotation().minus(desiredRotation).getRadians();
        double driveScale = Math.cos(angleError);
        // Only drive when generally facing the target
        driveScale = Math.max(0, driveScale);

        double finalDriveSpeed = driveSpeed * driveScale;

        // Apply max speed limits
        finalDriveSpeed = MathUtil.clamp(finalDriveSpeed, -0.5, 0.5);
        rotationSpeed = MathUtil.clamp(rotationSpeed, -0.5, 0.5);

        driveSubsystem.arcadeDrive(finalDriveSpeed, rotationSpeed);

        SmartDashboard.putString("AutoAlign/TargetID", bestTarget.get().scoringPose.id);
        SmartDashboard.putNumber("AutoAlign/DistanceError", currentDistance - AutoAlignConstants.DESIRED_DISTANCE_METERS);
        SmartDashboard.putNumber("AutoAlign/RotationError", currentPose.getRotation().getDegrees() - desiredRotation.getDegrees());
    }

    private Optional<TargetCost> findBestTarget() {
        List<TargetCost> potentialTargets = new ArrayList<>();
        Pose2d currentPose = localizationSubsystem.getPose();
        double velocity = driveSubsystem.getChassisSpeeds().vxMetersPerSecond;

        for (var scoringPose : VisionConstants.ALL_SCORING_POSES) {
            AlignmentCostUtil.calculateCost(scoringPose, currentPose, velocity, reefState)
                .ifPresent(potentialTargets::add);
        }

        if (potentialTargets.isEmpty()) {
            return Optional.empty();
        }

        Collections.sort(potentialTargets);
        return Optional.of(potentialTargets.get(0));
    }

    @Override
    public boolean isFinished() {
        // This command runs as long as the button is held, so it never "finishes" on its own.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    public Command getMarkScoredCommand() {
        return new InstantCommand(() -> {
            Optional<TargetCost> target = findBestTarget();
            target.ifPresent(t -> {
                reefState.markScored(t.scoringPose.id);
                System.out.println("Marked " + t.scoringPose.id + " as scored.");
            });
        });
    }
}
