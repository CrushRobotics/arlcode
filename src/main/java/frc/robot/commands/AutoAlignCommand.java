package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    private static Pose2d lockedTargetPose = null;
    private Optional<TargetCost> bestTarget = Optional.empty();
    private boolean isCloseToTarget = false;

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
        isCloseToTarget = false;
        if (lockedTargetId == null) {
            bestTarget = findBestTarget();
            if (bestTarget.isPresent()) {
                lockedTargetId = bestTarget.get().scoringPose.id;
                lockedTargetPose = bestTarget.get().targetPose;
                System.out.println("Auto Align Initialized: New target locked: " + lockedTargetId);
            } else {
                 System.out.println("Auto Align Initialized: No valid targets found.");
                 lockedTargetPose = null;
            }
        } else {
            System.out.println("Auto Align Re-initialized: Sticking with locked target: " + lockedTargetId);
            // If we have a locked target, we need to recalculate its pose for the current robot position
            Optional<TargetCost> lockedTargetOpt = findTargetById(lockedTargetId);
            if(lockedTargetOpt.isPresent()){
                bestTarget = lockedTargetOpt;
                lockedTargetPose = bestTarget.get().targetPose;
            } else {
                System.out.println("Could not re-find locked target. Clearing lock.");
                lockedTargetId = null; 
                lockedTargetPose = null;
            }
        }

        // Reset the PID controllers with the current measurements
        Pose2d currentPose = localizationSubsystem.getPose();
        if (lockedTargetPose != null) {
            double initialDistance = lockedTargetPose.getTranslation().minus(currentPose.getTranslation()).getNorm();
            driveController.reset(initialDistance);
        } else {
            driveController.reset(0);
        }
        turnController.reset(currentPose.getRotation().getDegrees());
    }

    @Override
    public void execute() {
        if (lockedTargetPose == null) {
            driveSubsystem.stop();
            return;
        }
    
        Pose2d currentPose = localizationSubsystem.getPose();
        // The target pose is now the locked pose
        Pose2d targetPose = lockedTargetPose;

        double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());

        // Phase 1: Vision-guided. If we're far away, keep updating the best target
        // in case a better one appears or our initial choice becomes invalid.
        if (!isCloseToTarget && currentDistance > AutoAlignConstants.FINAL_APPROACH_DISTANCE_METERS) {
            Optional<TargetCost> currentBest = findBestTarget();
            if (currentBest.isPresent()) {
                bestTarget = currentBest;
                lockedTargetId = bestTarget.get().scoringPose.id;
                lockedTargetPose = bestTarget.get().targetPose; // Update the locked pose
            } else {
                // We lost all targets, stop.
                lockedTargetPose = null;
                driveSubsystem.stop();
                return;
            }
        } else {
            // Phase 2: Odometry-guided. We are close, so commit to the last known pose.
            isCloseToTarget = true;
        }
        
        // --- Translation Control ---
        Translation2d translationToTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
        double driveSpeed = -driveController.calculate(currentDistance, 0);
        
        // --- Rotation Control ---
        Rotation2d desiredRotation = targetPose.getRotation();
        double rotationSpeedDegPerSec = turnController.calculate(
            currentPose.getRotation().getDegrees(),
            desiredRotation.getDegrees()
        );
    
        // --- Stopping Logic ---
        double rotationalErrorDegrees = currentPose.getRotation().minus(desiredRotation).getDegrees();
        if (driveController.atSetpoint() && Math.abs(rotationalErrorDegrees) < AutoAlignConstants.TURN_TOLERANCE_DEGREES) {
            driveSubsystem.stop();
            return;
        }
        
        // --- Command Motors ---
        Rotation2d travelDirection = translationToTarget.getAngle();
        double angleError = currentPose.getRotation().minus(travelDirection).getRadians();
        double driveScale = Math.cos(angleError);
        
        double finalDriveSpeed = driveSpeed * Math.max(0, driveScale);
        finalDriveSpeed = MathUtil.clamp(finalDriveSpeed, -0.7, 0.7);
    
        double rotationSpeedRadPerSec = Units.degreesToRadians(rotationSpeedDegPerSec);
        
        ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds(finalDriveSpeed, 0, rotationSpeedRadPerSec);
        driveSubsystem.setChassisSpeeds(targetChassisSpeeds);
    
        SmartDashboard.putString("AutoAlign/TargetID", lockedTargetId);
        SmartDashboard.putNumber("AutoAlign/DistanceError", currentDistance);
        SmartDashboard.putNumber("AutoAlign/RotationError", rotationalErrorDegrees);
        SmartDashboard.putBoolean("AutoAlign/IsCloseToTarget", isCloseToTarget);
    }

    private Optional<TargetCost> findBestTarget() {
        Set<Integer> visibleTagIds = visionSubsystem.getVisibleTagIds();
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (visibleTagIds.isEmpty() || alliance.isEmpty()) {
            System.out.println("AutoAlign: No tags visible or no alliance detected.");
            return Optional.empty();
        }

        List<TargetCost> potentialTargets = new ArrayList<>();
        Pose2d currentPose = localizationSubsystem.getPose();
        double velocity = driveSubsystem.getChassisSpeeds().vxMetersPerSecond;

        for (var scoringPoseInfo : VisionConstants.ALL_SCORING_POSES) {
            if (visibleTagIds.contains(scoringPoseInfo.parentTagId)) {
                // Dynamically calculate the pose for this target
                Optional<Pose2d> targetPoseOpt = VisionConstants.getFieldRelativePose(scoringPoseInfo, alliance.get());
                
                if (targetPoseOpt.isPresent()) {
                    TargetCost cost = AlignmentCostUtil.calculateCost(scoringPoseInfo, targetPoseOpt.get(), currentPose, velocity, reefState);
                    potentialTargets.add(cost);
                }
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
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isEmpty()) return Optional.empty();

        for (var scoringPoseInfo : VisionConstants.ALL_SCORING_POSES) {
            if (scoringPoseInfo.id.equals(id)) {
                Optional<Pose2d> targetPoseOpt = VisionConstants.getFieldRelativePose(scoringPoseInfo, alliance.get());
                if (targetPoseOpt.isPresent()) {
                    return Optional.of(AlignmentCostUtil.calculateCost(scoringPoseInfo, targetPoseOpt.get(), currentPose, velocity, reefState));
                }
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
            lockedTargetPose = null;
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

