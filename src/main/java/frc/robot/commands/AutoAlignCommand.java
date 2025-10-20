package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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
import java.util.Set;

public class AutoAlignCommand extends Command {

    private final CANDriveSubsystem driveSubsystem;
    private final LocalizationSubsystem localizationSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final ReefState reefState;

    private final ProfiledPIDController turnController;
    private final ProfiledPIDController driveController;

    private static String lockedTargetId = null;
    private Optional<TargetCost> bestTarget = Optional.empty();

    public AutoAlignCommand(CANDriveSubsystem drive, LocalizationSubsystem localization, VisionSubsystem vision, ReefState reef) {
        this.driveSubsystem = drive;
        this.localizationSubsystem = localization;
        this.visionSubsystem = vision;
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
        
        addRequirements(drive, localization, vision);
    }

    @Override
    public void initialize() {
        if (lockedTargetId == null) {
            bestTarget = findBestTarget();
            if (bestTarget.isPresent()) {
                lockedTargetId = bestTarget.get().scoringPose.id;
                System.out.println("Auto Align Initialized: New target locked: " + lockedTargetId);
            } else {
                 System.out.println("Auto Align Initialized: No valid targets found.");
            }
        } else {
            System.out.println("Auto Align Re-initialized: Sticking with locked target: " + lockedTargetId);
            bestTarget = findTargetById(lockedTargetId);
            if (bestTarget.isEmpty()) {
                System.out.println("Could not re-find locked target. Clearing lock.");
                lockedTargetId = null; 
            }
        }
    }

    @Override
    public void execute() {
        if (bestTarget.isEmpty()) {
            driveSubsystem.stop();
            return;
        }

        Pose2d currentPose = localizationSubsystem.getPose();
        Pose2d targetPose = bestTarget.get().scoringPose.pose;
        
        Translation2d translationToTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
        double currentDistance = translationToTarget.getNorm();
        Rotation2d desiredRotation = translationToTarget.getAngle();

        double driveSpeed = -driveController.calculate(currentDistance, AutoAlignConstants.DESIRED_DISTANCE_METERS);

        double angleError = currentPose.getRotation().minus(desiredRotation).getRadians();
        double driveScale = Math.cos(angleError);
        driveScale = Math.max(0, driveScale);
        double finalDriveSpeed = driveSpeed * driveScale;
        
        finalDriveSpeed = MathUtil.clamp(finalDriveSpeed, -1.0, 1.0);

        double rotationSpeedDegPerSec = turnController.calculate(
            currentPose.getRotation().getDegrees(),
            desiredRotation.getDegrees()
        );

        double rotationalErrorDegrees = currentPose.getRotation().minus(desiredRotation).getDegrees();

        if (driveController.atSetpoint() && Math.abs(rotationalErrorDegrees) < AutoAlignConstants.TURN_TOLERANCE_DEGREES) {
            driveSubsystem.stop();
            return;
        }

        double rotationSpeedRadPerSec = Units.degreesToRadians(rotationSpeedDegPerSec);
        
        ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds(finalDriveSpeed, 0, rotationSpeedRadPerSec);
        
        driveSubsystem.setChassisSpeeds(targetChassisSpeeds);

        SmartDashboard.putString("AutoAlign/TargetID", bestTarget.get().scoringPose.id);
        SmartDashboard.putNumber("AutoAlign/DistanceError", currentDistance - AutoAlignConstants.DESIRED_DISTANCE_METERS);
        SmartDashboard.putNumber("AutoAlign/RotationError", rotationalErrorDegrees);
    }

    private Optional<TargetCost> findBestTarget() {
        Set<Integer> visibleTagIds = visionSubsystem.getVisibleTagIds();

        if (visibleTagIds.isEmpty()) {
            System.out.println("AutoAlign: No tags visible to any camera.");
            return Optional.empty();
        }

        List<TargetCost> potentialTargets = new ArrayList<>();
        Pose2d currentPose = localizationSubsystem.getPose();
        double velocity = driveSubsystem.getChassisSpeeds().vxMetersPerSecond;

        for (var scoringPose : VisionConstants.ALL_SCORING_POSES) {
            if (visibleTagIds.contains(scoringPose.parentTagId)) {
                AlignmentCostUtil.calculateCost(scoringPose, currentPose, velocity, reefState)
                    .ifPresent(potentialTargets::add);
            }
        }

        if (potentialTargets.isEmpty()) {
            System.out.println("AutoAlign: Visible tags are not valid scoring targets.");
            return Optional.empty();
        }

        Collections.sort(potentialTargets);
        return Optional.of(potentialTargets.get(0));
    }

    private Optional<TargetCost> findTargetById(String id) {
        Pose2d currentPose = localizationSubsystem.getPose();
        double velocity = driveSubsystem.getChassisSpeeds().vxMetersPerSecond;

        for (var scoringPose : VisionConstants.ALL_SCORING_POSES) {
            if (scoringPose.id.equals(id)) {
                return AlignmentCostUtil.calculateCost(scoringPose, currentPose, velocity, reefState);
            }
        }
        return Optional.empty();
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
        if (interrupted) {
            lockedTargetId = null;
            System.out.println("Auto Align interrupted. Target lock released.");
        }
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

