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
import edu.wpi.first.wpilibj.Timer;
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
    private final Timer blindTimer = new Timer();


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
        blindTimer.stop();
        blindTimer.reset();
        
        // Find the best target at the start of the command.
        Optional<TargetCost> initialTarget = findBestTarget();

        if (initialTarget.isPresent()) {
            lockedTargetId = initialTarget.get().scoringPose.id;
            lockedTargetPose = initialTarget.get().targetPose;
            System.out.println("Auto Align Initialized: Target locked: " + lockedTargetId);
        } else {
             System.out.println("Auto Align Initialized: No valid targets found.");
             lockedTargetId = null;
             lockedTargetPose = null;
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
        // 1. Continuously update target pose if vision is available
        Optional<TargetCost> currentBestVisibleTarget = findBestTarget();
        boolean hasVision = currentBestVisibleTarget.isPresent();

        if (hasVision) {
            // If we can see a target, update our locked pose with the latest data and reset the safety timer.
            lockedTargetId = currentBestVisibleTarget.get().scoringPose.id;
            lockedTargetPose = currentBestVisibleTarget.get().targetPose;
            blindTimer.stop();
            blindTimer.reset();
        }
        // If we can't see a target, we just keep driving towards the last known lockedTargetPose.
    
        // 2. Check if we have a valid target pose to drive to.
        if (lockedTargetPose == null) {
            driveSubsystem.stop();
            return;
        }
    
        Pose2d currentPose = localizationSubsystem.getPose();
        double currentDistance = currentPose.getTranslation().getDistance(lockedTargetPose.getTranslation());
        
        // 3. Blind Safety Timer Logic
        // If we are close, have no vision, and the timer isn't running, start it.
        if (currentDistance < AutoAlignConstants.FINAL_APPROACH_DISTANCE_METERS && !hasVision && blindTimer.get() == 0) {
            blindTimer.start();
        }

        // --- PID Calculation ---
        Translation2d translationToTarget = lockedTargetPose.getTranslation().minus(currentPose.getTranslation());
        double driveSpeed = -driveController.calculate(currentDistance, 0);
        
        // --- Dynamic Rotation Control ---
        Rotation2d rotationToTarget = translationToTarget.getAngle();
        Rotation2d finalRotation = lockedTargetPose.getRotation();
        
        double turnSetpoint;
        if (currentDistance > AutoAlignConstants.ROTATION_SWAP_DISTANCE_METERS) {
            turnSetpoint = rotationToTarget.getDegrees();
        } else {
            turnSetpoint = finalRotation.getDegrees();
        }

        double rotationSpeedDegPerSec = turnController.calculate(currentPose.getRotation().getDegrees(), turnSetpoint);
    
        // --- Stopping Logic ---
        double rotationalErrorDegrees = currentPose.getRotation().minus(finalRotation).getDegrees();
        boolean isAligned = driveController.atSetpoint() && Math.abs(rotationalErrorDegrees) < AutoAlignConstants.TURN_TOLERANCE_DEGREES;
        boolean safetyTimeoutReached = blindTimer.get() > AutoAlignConstants.BLIND_SAFETY_TIMEOUT_SECONDS && blindTimer.get() != 0;

        if (isAligned || safetyTimeoutReached) {
            if (safetyTimeoutReached) {
                System.out.println("AutoAlign: Safety timeout reached. Forcing stop.");
            }
            driveSubsystem.stop();
            return;
        }
        
        // --- Command Motors ---
        double angleError = currentPose.getRotation().minus(rotationToTarget).getRadians();
        double driveScale = Math.cos(angleError);
        
        double finalDriveSpeed = driveSpeed * Math.max(0, driveScale);
        finalDriveSpeed = MathUtil.clamp(finalDriveSpeed, -0.7, 0.7);
    
        double rotationSpeedRadPerSec = Units.degreesToRadians(rotationSpeedDegPerSec);
        
        ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds(finalDriveSpeed, 0, rotationSpeedRadPerSec);
        driveSubsystem.setChassisSpeeds(targetChassisSpeeds);
    
        SmartDashboard.putString("AutoAlign/TargetID", lockedTargetId != null ? lockedTargetId : "None");
        SmartDashboard.putNumber("AutoAlign/DistanceError", currentDistance);
        SmartDashboard.putNumber("AutoAlign/RotationError", rotationalErrorDegrees);
        SmartDashboard.putBoolean("AutoAlign/VisionAvailable", hasVision);
        SmartDashboard.putNumber("AutoAlign/BlindTimer", blindTimer.get());
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
                // Pass the current robot pose to dynamically decide the target
                Optional<Pose2d> targetPoseOpt = VisionConstants.getFieldRelativePose(scoringPoseInfo, alliance.get(), alignMode, currentPose);
                
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
            if (lockedTargetId != null) {
                reefState.markScored(lockedTargetId);
                System.out.println("Marked " + lockedTargetId + " as scored.");
                lockedTargetId = null;
                lockedTargetPose = null;
            }
        });
    }
}

