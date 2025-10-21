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

        // Reset the PID controllers with the current measurements
        Pose2d currentPose = localizationSubsystem.getPose();
        if (bestTarget.isPresent()) {
            double initialDistance = bestTarget.get().targetPose.getTranslation().minus(currentPose.getTranslation()).getNorm();
            driveController.reset(initialDistance);
        } else {
            driveController.reset(0);
        }
        turnController.reset(currentPose.getRotation().getDegrees());
    }

    @Override
    public void execute() {
        if (bestTarget.isEmpty()) {
            driveSubsystem.stop();
            return;
        }
    
        Pose2d currentPose = localizationSubsystem.getPose();
        // The target pose is now dynamically calculated and stored in our bestTarget object
        Pose2d targetPose = bestTarget.get().targetPose;
        
        // --- Translation Control ---
        Translation2d translationToTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
        double currentDistance = translationToTarget.getNorm();
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
    
        SmartDashboard.putString("AutoAlign/TargetID", bestTarget.get().scoringPose.id);
        SmartDashboard.putNumber("AutoAlign/DistanceError", currentDistance);
        SmartDashboard.putNumber("AutoAlign/RotationError", rotationalErrorDegrees);
    }

    private Optional<TargetCost> findBestTarget() {
        Optional<Alliance> allianceOpt = DriverStation.getAlliance();
        if (allianceOpt.isEmpty()) {
            System.out.println("AutoAlign: No alliance color found.");
            return Optional.empty();
        }
        Alliance alliance = allianceOpt.get();

        Set<Integer> visibleTagIds = visionSubsystem.getVisibleTagIds();
        if (visibleTagIds.isEmpty()) {
            System.out.println("AutoAlign: No tags visible to any camera.");
            return Optional.empty();
        }

        List<TargetCost> potentialTargets = new ArrayList<>();
        Pose2d currentPose = localizationSubsystem.getPose();
        double velocity = driveSubsystem.getChassisSpeeds().vxMetersPerSecond;

        for (var scoringPoseInfo : VisionConstants.ALL_SCORING_POSES) {
            if (visibleTagIds.contains(scoringPoseInfo.parentTagId)) {
                // Dynamically calculate the field-relative pose for this target
                Optional<Pose2d> targetPoseOpt = VisionConstants.getFieldRelativePose(scoringPoseInfo, alliance);
                
                targetPoseOpt.ifPresent(targetPose -> {
                    // If pose is valid, calculate its cost and add it to our list
                    TargetCost cost = AlignmentCostUtil.calculateCost(
                        scoringPoseInfo, targetPose, currentPose, velocity, reefState
                    );
                    potentialTargets.add(cost);
                });
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
        Optional<Alliance> allianceOpt = DriverStation.getAlliance();
        if (allianceOpt.isEmpty()) return Optional.empty();
        Alliance alliance = allianceOpt.get();

        Pose2d currentPose = localizationSubsystem.getPose();
        double velocity = driveSubsystem.getChassisSpeeds().vxMetersPerSecond;

        for (var scoringPoseInfo : VisionConstants.ALL_SCORING_POSES) {
            if (scoringPoseInfo.id.equals(id)) {
                Optional<Pose2d> targetPoseOpt = VisionConstants.getFieldRelativePose(scoringPoseInfo, alliance);
                
                // If the pose can be calculated, create and return a TargetCost object for it
                return targetPoseOpt.map(targetPose -> 
                    AlignmentCostUtil.calculateCost(scoringPoseInfo, targetPose, currentPose, velocity, reefState)
                );
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

