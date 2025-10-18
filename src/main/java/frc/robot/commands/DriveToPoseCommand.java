package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LocalizationConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.TankDriveSubsystem;

import java.util.List;

/**
 * Drives the robot from its current pose to a target pose using WPILib's
 * TrajectoryGenerator + RamseteController, and feeds the resulting ChassisSpeeds
 * directly into TankDriveSubsystem.setChassisSpeeds().
 *
 * Robot will come to a stop at the end pose (0 start/end velocities).
 */
public class DriveToPoseCommand extends Command {
  private final TankDriveSubsystem drive;
  private final Pose2d endPose;

  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(LocalizationConstants.TRACK_WIDTH_METERS);

  private final RamseteController ramsete = new RamseteController(); // defaults: b=2.0, zeta=0.7
  private Trajectory trajectory;
  private final Timer timer = new Timer();

  public DriveToPoseCommand(TankDriveSubsystem drive, Pose2d endPose) {
    this.drive = drive;
    this.endPose = endPose;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    var start = drive.getPose();

    // Trajectory config using drivetrain limits from Constants
    TrajectoryConfig cfg = new TrajectoryConfig(
        TrajectoryConstants.CRUISE_MPS,
        TrajectoryConstants.ACCEL_MPS2
    ).setKinematics(kinematics);

    // Stop at the end (and start). This helps staging precisely.
    cfg.setStartVelocity(0.0);
    cfg.setEndVelocity(0.0);

    // No interior waypoints for simplicity; a pure start->end quintic spline
    trajectory = TrajectoryGenerator.generateTrajectory(
        start,
        List.of(), // No interior waypoints
        endPose,
        cfg
    );

    timer.reset();
    timer.start();

    SmartDashboard.putString("DriveToPose/Start", start.toString());
    SmartDashboard.putString("DriveToPose/End", endPose.toString());
    SmartDashboard.putNumber("DriveToPose/TotalTime", trajectory.getTotalTimeSeconds());
  }

  @Override
  public void execute() {
    double t = timer.get();
    Trajectory.State ref = trajectory.sample(t);

    // Ramsete yields desired ChassisSpeeds (vx, 0, omega) for diff drive
    ChassisSpeeds speeds = ramsete.calculate(drive.getPose(), ref);
    drive.setChassisSpeeds(speeds);

    SmartDashboard.putNumber("DriveToPose/Time", t);
    SmartDashboard.putString("DriveToPose/RefPose", ref.poseMeters.toString());
    SmartDashboard.putNumber("DriveToPose/CmdVx", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("DriveToPose/CmdOmega", speeds.omegaRadiansPerSecond);
  }

  @Override
  public boolean isFinished() {
    // End when trajectory time has elapsed; Ramsete converges near the end
    return timer.get() >= trajectory.getTotalTimeSeconds();
  }

  @Override
  public void end(boolean interrupted) {
    drive.setChassisSpeeds(new ChassisSpeeds()); // stop
    SmartDashboard.putBoolean("DriveToPose/Interrupted", interrupted);
  }
}
