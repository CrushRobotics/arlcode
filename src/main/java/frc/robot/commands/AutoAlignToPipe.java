package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.subsystems.*;
import frc.robot.AutoHelpers;
import frc.robot.AutoHelpers.*;
import java.util.List;

import dev.doglog.DogLog;

/**
 * AutoAlign (trajectory version):
 *  1) Pick closest reef pipe base pose (alliance-aware).
 *  2) Build approach pose = 1.0 m behind final stop along peg +X (front scoring).
 *  3) DriveToPose(current -> approach), stop.
 *  4) DriveToPose(approach -> final stop), stop.
 *
 * This keeps heading embedded in the poses (TrajectoryGenerator will interpolate
 * heading from current to goal). Stopping between legs helps staging.
 */
public class AutoAlignToPipe extends SequentialCommandGroup {



  Pose2d basePose = Pose2d.kZero;
  Pose2d finalPose = Pose2d.kZero;
  Pose2d approachPose = Pose2d.kZero;
  CANDriveSubsystem drive;

  public AutoAlignToPipe(CANDriveSubsystem drive, String pipeLetter) {
    this.drive = drive;
    boolean red  = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                    == DriverStation.Alliance.Red;
    PipeBase best = null;
    for (PipeBase p : AutoHelpers.PIPES) {
      if (p.name().equals(pipeLetter)) {
        best = p;
      }
    }
    // Compute plan now (so the sequence is fixed when scheduled)
    basePose = red ? best.red() : best.blue();

    // Final stop: DESIRED_DISTANCE in front of peg (i.e., along +X of peg frame into the reef face)
    finalPose = basePose.transformBy(new Transform2d(
        new Translation2d(AutoHelpers.kScoreOffset.get(), AutoHelpers.kArmOffset.get()),
        Rotation2d.kZero));
    finalPose = new Pose2d(finalPose.getTranslation(), basePose.getRotation()); // enforce heading

    // Approach: 1.0 m further back along -X of peg
    approachPose = finalPose.transformBy(new Transform2d(
        new Translation2d(AutoHelpers.kApproachOffset.get(), 0.0),
        Rotation2d.kZero));
    approachPose = new Pose2d(approachPose.getTranslation(), basePose.getRotation()); // enforce heading

    DogLog.log("AutoAlign/Pipe", best.name());
    DogLog.log("AutoAlign/Base", basePose);
    DogLog.log("AutoAlign/Approach", approachPose);
    DogLog.log("AutoAlign/Final", finalPose);
    // Simple chain: current -> approach -> final
    addCommands(
        Commands.runOnce(()-> {
          DogLog.log("AutoAlign/Base", basePose);
          DogLog.log("AutoAlign/Approach", approachPose);
          DogLog.log("AutoAlign/Final", finalPose);
        }),
        new DriveToPose(drive, ()-> approachPose, ()-> AutoHelpers.kApproachLinearTolerance, ()-> AutoHelpers.kApproachAngularTolerance, false),
        new DriveToPose(drive, ()-> finalPose, ()-> AutoHelpers.kScoreLinearTolerance, ()-> AutoHelpers.kScoreAngularTolerance, false)
    );
  }
}
