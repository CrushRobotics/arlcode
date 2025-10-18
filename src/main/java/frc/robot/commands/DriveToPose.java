package frc.robot.commands;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TankDrive;

import java.util.List;

/**
 * Drives the robot from its current pose to a target pose using WPILib's
 * TrajectoryGenerator + RamseteController, and feeds the resulting ChassisSpeeds
 * directly into TankDrive.setChassisSpeeds().
 *
 * Robot will come to a stop at the end pose (0 start/end velocities).
 */
public class DriveToPose extends Command {
  private final TankDrive drive;
  private final Pose2d endPose;

  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(Constants.DriveConstants.TRACK_WIDTH_METERS);

  private final RamseteController ramsete = new RamseteController(); // defaults: b=2.0, zeta=0.7
  private Trajectory trajectory;
  private final Timer timer = new Timer();

  public DriveToPose(TankDrive drive, Pose2d endPose) {
    this.drive = drive;
    this.endPose = endPose;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    var start = drive.getPose();

    // Trajectory config using drivetrain limits
    TrajectoryConfig cfg = new TrajectoryConfig(
        Constants.DriveConstants.CRUISE_MPS,
        Constants.DriveConstants.ACCEL_MPS2
    ).setKinematics(kinematics);

    // Stop at the end (and start). This helps staging precisely.
    cfg.setStartVelocity(0.0);
    cfg.setEndVelocity(0.0);

    // No interior waypoints for simplicity; a pure start->end quintic
    trajectory = TrajectoryGenerator.generateTrajectory(
        start,
        List.of(),
        endPose,
        cfg
    );

    timer.reset();
    timer.start();

    DogLog.log("DriveToPose/Start", start);
    DogLog.log("DriveToPose/End", endPose);
    DogLog.log("DriveToPose/TotalTime", trajectory.getTotalTimeSeconds());
  }

  @Override
  public void execute() {
    double t = timer.get();
    Trajectory.State ref = trajectory.sample(t);

    // Ramsete yields desired ChassisSpeeds (vx, 0, omega) for diff drive
    ChassisSpeeds speeds = ramsete.calculate(drive.getPose(), ref);
    drive.setChassisSpeeds(speeds);

    DogLog.log("DriveToPose/t", t);
    DogLog.log("DriveToPose/RefPose", ref.poseMeters);
    DogLog.log("DriveToPose/CmdVx", speeds.vxMetersPerSecond);
    DogLog.log("DriveToPose/CmdOmega", speeds.omegaRadiansPerSecond);
  }

  @Override
  public boolean isFinished() {
    // End when trajectory time has elapsed; Ramsete converges near the end
    return timer.get() >= trajectory.getTotalTimeSeconds();
  }

  @Override
  public void end(boolean interrupted) {
    drive.setChassisSpeeds(new ChassisSpeeds()); // stop
    DogLog.log("DriveToPose/Interrupted", interrupted);
  }
}
