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
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CANDriveSubsystem; // Needed to set speeds
import frc.robot.subsystems.LocalizationSubsystem; // Needed to get fused pose

import java.util.List;

/**
 * Drives the robot from a given start pose to a target end pose using WPILib's
 * TrajectoryGenerator + RamseteController.
 *
 * It gets the current pose from LocalizationSubsystem and commands CANDriveSubsystem.
 * Robot will come to a stop at the end pose (0 start/end velocities enforced).
 * Assumes the start pose provided accurately reflects the robot's position when the trajectory is generated.
 */
public class DriveToPoseTrajectoryCommand extends Command {
  private final CANDriveSubsystem driveSubsystem;
  private final LocalizationSubsystem localizationSubsystem; // Added
  private final Pose2d startPose; // Provided start pose (used for generation)
  private final Pose2d endPose;   // Target end pose

  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH_METERS);

  // RamseteController helps follow the trajectory using odometry feedback
  private final RamseteController ramseteController = new RamseteController();

  private Trajectory trajectory;
  private final Timer timer = new Timer();

  /**
   * Creates a command to drive from a start Pose2d to an end Pose2d using a trajectory.
   * @param drive The drive subsystem.
   * @param localization The localization subsystem.
   * @param start The starting pose (used for trajectory generation).
   * @param end The target ending pose.
   */
  public DriveToPoseTrajectoryCommand(CANDriveSubsystem drive, LocalizationSubsystem localization, Pose2d start, Pose2d end) {
    this.driveSubsystem = drive;
    this.localizationSubsystem = localization; // Store localization
    this.startPose = start;
    this.endPose = end;
    addRequirements(drive, localization); // Add localization as requirement
  }

  @Override
  public void initialize() {
    // 1. Create Trajectory Configuration
    TrajectoryConfig config = new TrajectoryConfig(
        DriveConstants.MAX_SPEED_MPS,
        DriveConstants.MAX_ACCELERATION_MPS_SQ
    ).setKinematics(kinematics);

    // Enforce stopping at the start and end of this segment
    config.setStartVelocity(0.0);
    config.setEndVelocity(0.0);

    // 2. Generate the Trajectory
    // Uses the provided start pose for generation. Ramsete will correct based on actual current pose during execution.
    try {
        trajectory = TrajectoryGenerator.generateTrajectory(
            startPose,
            List.of(), // No interior waypoints
            endPose,
            config
        );
    } catch (Exception e) {
        System.err.println("!!!!!!!!!! Trajectory Generation Failed !!!!!!!!!!");
        System.err.println("Start Pose: " + startPose);
        System.err.println("End Pose: " + endPose);
        e.printStackTrace();
        // Set a dummy trajectory to prevent null pointers, command will end immediately
        trajectory = TrajectoryGenerator.generateTrajectory(startPose, List.of(), startPose, config);
    }


    // 3. Reset Timer and Start
    timer.reset();
    timer.start();

    // Logging for debug
    SmartDashboard.putString("DriveToPoseTraj/Start", startPose.toString());
    SmartDashboard.putString("DriveToPoseTraj/End", endPose.toString());
    SmartDashboard.putNumber("DriveToPoseTraj/TotalTime", trajectory.getTotalTimeSeconds());
    SmartDashboard.putString("DriveToPoseTraj/Status", "Running");
  }

  @Override
  public void execute() {
    double currentTime = timer.get();
    Trajectory.State desiredState = trajectory.sample(currentTime);

    // Get current pose from LocalizationSubsystem
    Pose2d currentActualPose = localizationSubsystem.getPose();

    // Use RamseteController to calculate desired chassis speeds based on actual current pose and trajectory state
    ChassisSpeeds targetChassisSpeeds = ramseteController.calculate(currentActualPose, desiredState);

    // Command the drive subsystem with calculated speeds
    driveSubsystem.setChassisSpeeds(targetChassisSpeeds);

    // Logging for debug
    SmartDashboard.putNumber("DriveToPoseTraj/Time", currentTime);
    SmartDashboard.putString("DriveToPoseTraj/RefPose", desiredState.poseMeters.toString());
    SmartDashboard.putNumber("DriveToPoseTraj/RefVel", desiredState.velocityMetersPerSecond);
    SmartDashboard.putString("DriveToPoseTraj/ActualPose", currentActualPose.toString()); // Log actual pose
    SmartDashboard.putNumber("DriveToPoseTraj/CmdVx", targetChassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("DriveToPoseTraj/CmdOmega", targetChassisSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("DriveToPoseTraj/PoseErrorX", currentActualPose.getX() - desiredState.poseMeters.getX());
    SmartDashboard.putNumber("DriveToPoseTraj/PoseErrorY", currentActualPose.getY() - desiredState.poseMeters.getY());
    SmartDashboard.putNumber("DriveToPoseTraj/PoseErrorRot", currentActualPose.getRotation().minus(desiredState.poseMeters.getRotation()).getDegrees());

  }

  @Override
  public boolean isFinished() {
    // Finish command once the trajectory time has been exceeded
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain when the command ends or is interrupted
    driveSubsystem.stop();
    timer.stop();
    SmartDashboard.putString("DriveToPoseTraj/Status", interrupted ? "Interrupted" : "Finished");
    System.out.println("DriveToPoseTrajectoryCommand ended. Interrupted: " + interrupted);
  }
}

