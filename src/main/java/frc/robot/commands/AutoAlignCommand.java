package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ReefState;
import frc.robot.subsystems.TankDriveSubsystem;
import frc.robot.Constants.VisionConstants.ScoringPose;

import java.util.Comparator;
import java.util.Optional;

/**
 * AutoAlign (trajectory version):
 * 1) Pick closest valid reef scoring pose (not scored yet).
 * 2) Build an "approach" pose = 1.0 m behind the final scoring pose.
 * 3) Schedules a DriveToPose to go from current -> approach, then stop.
 * 4) Schedules a second DriveToPose to go from approach -> final stop.
 */
public class AutoAlignCommand extends Command {

  private final TankDriveSubsystem drive;
  private final ReefState reefState;
  private Command autoCommand; // The sequential command group we will run

  public AutoAlignCommand(TankDriveSubsystem drive, ReefState reefState) {
    this.drive = drive;
    this.reefState = reefState;
    addRequirements(drive);
  }
  
  @Override
  public void initialize() {
    // Dynamically build the command sequence each time this command starts.
    Optional<ScoringPose> bestTarget = findBestTarget(drive.getPose());

    if (bestTarget.isEmpty()) {
        SmartDashboard.putString("AutoAlign/TargetID", "None");
        System.out.println("AutoAlign: No valid target found.");
        autoCommand = null; // No command to run
        return;
    }

    ScoringPose target = bestTarget.get();
    Pose2d finalStop = target.pose;

    // Approach pose is 1 meter behind the final stop pose, along the same heading.
    Pose2d approach = finalStop.transformBy(new Transform2d(
        new Translation2d(-1.0, 0.0),
        new Rotation2d()
    ));

    SmartDashboard.putString("AutoAlign/TargetID", target.id);
    SmartDashboard.putString("AutoAlign/Approach", approach.toString());
    SmartDashboard.putString("AutoAlign/Final", finalStop.toString());

    // Create the two-part trajectory drive as a sequential command group
    autoCommand = new SequentialCommandGroup(
        new DriveToPoseCommand(drive, approach),
        new DriveToPoseCommand(drive, finalStop)
    );
    
    // Initialize the created command
    autoCommand.initialize();
  }

  @Override
  public void execute() {
      if (autoCommand != null) {
          autoCommand.execute();
      }
  }

  @Override
  public void end(boolean interrupted) {
      if(autoCommand != null) {
          autoCommand.end(interrupted);
      }
  }

  @Override
  public boolean isFinished() {
      // The command is finished if no path was generated or the path is done
      return autoCommand == null || autoCommand.isFinished();
  }

  private Optional<ScoringPose> findBestTarget(Pose2d currentPose) {
    // Find the closest scoring pose that has not already been scored on
    return frc.robot.Constants.VisionConstants.ALL_SCORING_POSES.stream()
        .filter(p -> !reefState.isScored(p.id)) // Filter out scored poses
        .min(Comparator.comparingDouble(p -> 
            p.pose.getTranslation().getDistance(currentPose.getTranslation())));
  }

  /**
   * Returns an InstantCommand that marks the currently targeted scoring location as scored.
   */
  public InstantCommand getMarkScoredCommand() {
      return new InstantCommand(() -> {
          // Re-evaluate the best target when the button is pressed, in case the robot has moved
          findBestTarget(drive.getPose()).ifPresent(t -> {
              reefState.markScored(t.id);
              System.out.println("Marked " + t.id + " as scored.");
              SmartDashboard.putBoolean("ReefState/" + t.id, true);
          });
      }, reefState); // Add subsystem requirement
  }
}

