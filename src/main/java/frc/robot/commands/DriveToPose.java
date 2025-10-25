package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoHelpers;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CANDriveSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import dev.doglog.DogLog;

/**
 * Drives the robot from its current pose to a target pose using WPILib's
 * TrajectoryGenerator + RamseteController, and feeds the resulting ChassisSpeeds
 * directly into TankDrive.setChassisSpeeds().
 *
 * Robot will come to a stop at the end pose (0 start/end velocities).
 */
public class DriveToPose extends Command {
  private final CANDriveSubsystem drive;
  private final Supplier<Pose2d> endPose;
  private final Supplier<Double> linearTolerance;
  private final Supplier<Double> angularTolerance;

  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(Constants.DriveConstants.TRACK_WIDTH_METERS);

  private final RamseteController ramsete = new RamseteController(); // defaults: b=2.0, zeta=0.7
  private Trajectory trajectory;
  private final Timer timer = new Timer();

  private final Debouncer debouncer = new Debouncer(0.25);

  private boolean failedToGenerate = false;
  private boolean isReversed = false;

  public DriveToPose(CANDriveSubsystem drive, Supplier<Pose2d> endPose, Supplier<Double> linearTolerance, Supplier<Double> angularTolerance, boolean isReversed) {
    this.drive = drive;
    this.endPose = endPose;
    this.linearTolerance = linearTolerance;
    this.angularTolerance = angularTolerance;
    this.isReversed = isReversed;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    failedToGenerate = false;
    var start = drive.getLocalizationSubsystem().getPose();
    // Trajectory config using drivetrain limits
    TrajectoryConfig cfg = new TrajectoryConfig(
        AutoHelpers.kMaxLinearVelocity,
        AutoHelpers.kMaxLinearAcceleration
    ).setKinematics(kinematics).addConstraint(new DifferentialDriveKinematicsConstraint(kinematics, AutoHelpers.kMaxLinearVelocityPerSide))
    .addConstraint(new CentripetalAccelerationConstraint(AutoHelpers.kMaxLinearAcceleration))
    .setReversed(isReversed);

    // Stop at the end (and start). This helps staging precisely.
    cfg.setStartVelocity(drive.getChassisSpeeds().vxMetersPerSecond);
    cfg.setEndVelocity(0.0);

    // No interior waypoints for simplicity; a pure start->end quintic
    try {
      trajectory = TrajectoryGenerator.generateTrajectory(
          start,
          List.of(),
          endPose.get(),
          cfg
      );
    }
    catch (Exception e) {
      DogLog.timestamp("Failed to generate trajectory");
      failedToGenerate = true;
    }
    DogLog.log("/DriveToPose/Start", start);
    DogLog.log("DriveToPose/End", endPose.get());
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if (failedToGenerate) {
      return;
    }
    double t = timer.get();
    Trajectory.State ref = trajectory.sample(t);

    DogLog.log("DriveToPose/Goal", ref.poseMeters);

    // Ramsete yields desired ChassisSpeeds (vx, 0, omega) for diff drive
    ChassisSpeeds speeds = ramsete.calculate(drive.getLocalizationSubsystem().getPose(), ref);
    drive.setChassisSpeeds(speeds);
  }

  @Override
  public boolean isFinished() {
    // End when trajectory time has elapsed; Ramsete converges near the end
    return debouncer.calculate(drive.isNear(endPose.get(), linearTolerance.get(), angularTolerance.get())) || failedToGenerate || timer.get() >= trajectory.getTotalTimeSeconds() * 1.1;
  }

  @Override
  public void end(boolean interrupted) {
    drive.setChassisSpeeds(new ChassisSpeeds()); // stop
  }
}
