package frc.robot;

import java.util.List;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;

public class AutoHelpers {

  public static final double kMaxLinearVelocity = 3.5;
  public static final double kMaxLinearAcceleration = 2.5;
  public static final double kMaxLinearVelocityPerSide = 3.5;

  public static final DoubleSubscriber kApproachOffset = DogLog.tunable("AutoAlign/Approach Offset", -0.7);
  public static final DoubleSubscriber kScoreOffset = DogLog.tunable("AutoAlign/Score Offset", -Units.inchesToMeters(19.5 + 11 + 1));
  public static final DoubleSubscriber kArmOffset = DogLog.tunable("AutoAlign/Arm Offset", -Units.inchesToMeters(2.5));

  public static final double kApproachLinearTolerance = 0.2;
  public static final double kApproachAngularTolerance = 10;
  public static final double kScoreLinearTolerance = 0.03;
  public static final double kScoreAngularTolerance = 2;

  public static final Pose2d csLeftRed = new Pose2d(16, 0.61, Rotation2d.fromDegrees(120));
  public static final Pose2d csRightRed = new Pose2d(16.00, 7.36, Rotation2d.fromDegrees(-120));
  public static final Pose2d csLeftBlue = new Pose2d(1.5, 7.36, Rotation2d.fromDegrees(-60));
  public static final Pose2d csRightBlue = new Pose2d(1.5, 0.61, Rotation2d.fromDegrees(60));
      // Pipe base poses (front-scoring) for BLUE and RED
  public record PipeBase(String name, Pose2d blue, Pose2d red) {}

  public static final List<PipeBase> PIPES = List.of(
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
}
