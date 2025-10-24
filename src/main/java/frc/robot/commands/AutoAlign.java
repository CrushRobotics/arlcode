package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.subsystems.*;

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
public class AutoAlign extends SequentialCommandGroup {

  // Pipe base poses (front-scoring) for BLUE and RED
  private record PipeBase(String name, Pose2d blue, Pose2d red) {}

  private static final List<PipeBase> PIPES = List.of(
      new PipeBase("A", new Pose2d(3.71, 4.19, Rotation2d.kZero),
                         new Pose2d(13.84, 3.86, Rotation2d.k180deg)),
      new PipeBase("B", new Pose2d(3.71, 3.86, Rotation2d.kZero),
                         new Pose2d(13.84, 4.19, Rotation2d.k180deg)),
      new PipeBase("C", new Pose2d(3.96, 3.43, Rotation2d.fromDegrees(60)),
                         new Pose2d(13.59, 4.62, Rotation2d.fromDegrees(240))),
      new PipeBase("D", new Pose2d(4.24, 3.27, Rotation2d.fromDegrees(60)),
                         new Pose2d(13.31, 4.78, Rotation2d.fromDegrees(240))),
      new PipeBase("E", new Pose2d(4.74, 3.27, Rotation2d.fromDegrees(120)),
                         new Pose2d(12.81, 4.78, Rotation2d.fromDegrees(300))),
      new PipeBase("F", new Pose2d(5.02, 3.43, Rotation2d.fromDegrees(120)),
                         new Pose2d(12.53, 4.62, Rotation2d.fromDegrees(300))),
      new PipeBase("G", new Pose2d(5.27, 3.86, Rotation2d.k180deg),
                         new Pose2d(12.29, 4.19, Rotation2d.kZero)),
      new PipeBase("H", new Pose2d(5.27, 4.19, Rotation2d.k180deg),
                         new Pose2d(12.29, 3.86, Rotation2d.kZero)),
      new PipeBase("I", new Pose2d(5.02, 4.62, Rotation2d.fromDegrees(240)),
                         new Pose2d(12.53, 3.43, Rotation2d.fromDegrees(60))),
      new PipeBase("J", new Pose2d(4.74, 4.78, Rotation2d.fromDegrees(240)),
                         new Pose2d(12.81, 3.27, Rotation2d.fromDegrees(60))),
      new PipeBase("K", new Pose2d(4.24, 4.78, Rotation2d.fromDegrees(300)),
                         new Pose2d(13.31, 3.27, Rotation2d.fromDegrees(120))),
      new PipeBase("L", new Pose2d(3.96, 4.62, Rotation2d.fromDegrees(300)),
                         new Pose2d(13.59, 3.43, Rotation2d.fromDegrees(120)))
  );

  public AutoAlign(CANDriveSubsystem drive) {
    // Compute plan now (so the sequence is fixed when scheduled)
    Pose2d robot = drive.getLocalizationSubsystem().getPose();
    boolean red  = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                    == DriverStation.Alliance.Red;

    // Pick the closest pipe (by translation distance)
    PipeBase best = null;
    double bestDist = Double.POSITIVE_INFINITY;
    for (PipeBase p : PIPES) {
      Pose2d base = red ? p.red() : p.blue();
      double d = base.getTranslation().getDistance(robot.getTranslation());
      if (d < bestDist) { bestDist = d; best = p; }
    }

    if (best == null) {
      // If for some reason we don't have any pipes (shouldn't happen), do nothing.
      addCommands(); // empty
      return;
    }

    Pose2d basePose = red ? best.red() : best.blue();

    // Final stop: DESIRED_DISTANCE in front of peg (i.e., along +X of peg frame into the reef face)
    Pose2d finalStop = basePose.transformBy(new Transform2d(
        new Translation2d(-1.75, 0.0),
        Rotation2d.kZero));
    finalStop = new Pose2d(finalStop.getTranslation(), basePose.getRotation()); // enforce heading

    // Approach: 1.0 m further back along -X of peg
    Pose2d approach = finalStop.transformBy(new Transform2d(
        new Translation2d(-0.5, 0.0),
        Rotation2d.kZero));
    approach = new Pose2d(approach.getTranslation(), basePose.getRotation()); // enforce heading

    // Simple chain: current -> approach -> final
    addCommands(
        new DriveToPose(drive, approach),
        new DriveToPose(drive, finalStop)
    );
    DogLog.log("AutoAlign/HasTarget", true);
    DogLog.log("AutoAlign/Pipe", best.name());
    DogLog.log("AutoAlign/Base", basePose);
    DogLog.log("AutoAlign/Approach", approach);
    DogLog.log("AutoAlign/Final", finalStop);
  }
}
