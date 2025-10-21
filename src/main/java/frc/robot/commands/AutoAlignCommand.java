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

    /**
     * Defines the purpose of the alignment, which determines the robot's final orientation.
     */
    public enum AlignMode {
        SCORING,
        COLLECTING
    }

    private final CANDriveSubsystem driveSubsystem;
    private final LocalizationSubsystem localizationSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final ReefState reefState;
    private final AlignMode alignMode;

    private final ProfiledPIDController turnController;
    private final ProfiledPIDController driveController;

    private static String lockedTargetId = null;
    private static Pose2d lockedTargetPose = null;
    private Optional<TargetCost> bestTarget = Optional.empty();
    private boolean isCloseToTarget = false;

    public AutoAlignCommand(CANDriveSubsystem drive, LocalizationSubsystem localization, VisionSubsystem vision, ReefState reef, AlignMode mode) {
        this.driveSubsystem = drive;
        this.localizationSubsystem = localization;
        this.visionSubsystem = vision;
        this.reefState = reef;
        this.alignMode = mode;

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
        // If we don't have a lock, find the best target.
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
            // If we already have a lock, just recalculate the pose for it.
            System.out.println("Auto Align Re-initialized: Sticking with locked target: " + lockedTargetId);
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
        double currentDistance = currentPose.getTranslation().getDistance(lockedTargetPose.getTranslation());

        // Check if we have entered the final approach phase.
        if (currentDistance < AutoAlignConstants.FINAL_APPROACH_DISTANCE_METERS) {
            isCloseToTarget = true;
        }

        // --- Vision Update Logic ---
        // If we are NOT in the final approach, continuously look for a better target.
        if (!isCloseToTarget) {
            Optional<TargetCost> currentBest = findBestTarget();
            // If we see a target, update our lock. If we don't see one, we KEEP our old lock.
            if (currentBest.isPresent()) {
                bestTarget = currentBest;
                lockedTargetId = bestTarget.get().scoringPose.id;
                lockedTargetPose = bestTarget.get().targetPose; 
            }
        }
        
        // --- PID Calculation ---
        Translation2d translationToTarget = lockedTargetPose.getTranslation().minus(currentPose.getTranslation());
        double driveSpeed = -driveController.calculate(currentDistance, 0);
        
        // --- Dynamic Rotation Control ---
        Rotation2d rotationToTarget = translationToTarget.getAngle();
        Rotation2d finalRotation = lockedTargetPose.getRotation();
        
        double turnSetpoint;
        // When far, point towards the target. When close, point to the final scoring rotation.
        if (currentDistance > AutoAlignConstants.ROTATION_SWAP_DISTANCE_METERS) {
            turnSetpoint = rotationToTarget.getDegrees();
        } else {
            turnSetpoint = finalRotation.getDegrees();
        }

        double rotationSpeedDegPerSec = turnController.calculate(currentPose.getRotation().getDegrees(), turnSetpoint);
    
        // --- Stopping Logic ---
        double rotationalErrorDegrees = currentPose.getRotation().minus(finalRotation).getDegrees();
        if (driveController.atSetpoint() && Math.abs(rotationalErrorDegrees) < AutoAlignConstants.TURN_TOLERANCE_DEGREES) {
            driveSubsystem.stop();
            return;
        }
        
        // --- Command Motors ---
        // Scale forward speed so the robot doesn't try to drive sideways.
        double angleError = currentPose.getRotation().minus(rotationToTarget).getRadians();
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
            return Optional.empty();
        }

        List<TargetCost> potentialTargets = new ArrayList<>();
        Pose2d currentPose = localizationSubsystem.getPose();
        double velocity = driveSubsystem.getChassisSpeeds().vxMetersPerSecond;

        for (var scoringPoseInfo : VisionConstants.ALL_SCORING_POSES) {
            if (visibleTagIds.contains(scoringPoseInfo.parentTagId)) {
                Optional<Pose2d> targetPoseOpt = VisionConstants.getFieldRelativePose(scoringPoseInfo, alliance.get(), alignMode);
                
                if (targetPoseOpt.isPresent()) {
                    TargetCost cost = AlignmentCostUtil.calculateCost(scoringPoseInfo, targetPoseOpt.get(), currentPose, velocity, reefState);
                    potentialTargets.add(cost);
                }
            }
        }

        if (potentialTargets.isEmpty()) {
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
                Optional<Pose2d> targetPoseOpt = VisionConstants.getFieldRelativePose(scoringPoseInfo, alliance.get(), alignMode);
                if (targetPoseOpt.isPresent()) {
                    return Optional.of(AlignmentCostUtil.calculateCost(scoringPoseInfo, targetPoseOpt.get(), currentPose, velocity, reefState));
                }
            }
        }
        return Optional.empty();
    }


    @Override
    public boolean isFinished() {
        // This command will finish when the robot is at the setpoint, checked in execute().
        // For teleop control, we want it to run as long as the button is held, so we return false.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
        // We only clear the lock if the command was interrupted (e.g., driver let go of the button).
        // If it finishes normally by reaching the setpoint, we might want to keep the lock.
        if (interrupted) {
            lockedTargetId = null;
            lockedTargetPose = null;
            System.out.println("Auto Align interrupted. Target lock released.");
        }
    }

    public Command getMarkScoredCommand() {
        return new InstantCommand(() -> {
            if (lockedTargetId != null) {
                reefState.markScored(lockedTargetId);
                System.out.println("Marked " + lockedTargetId + " as scored.");
                // Clear the lock so the next alignment finds a new target.
                lockedTargetId = null;
                lockedTargetPose = null;
            }
        });
    }
}

