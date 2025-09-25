package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.BestTarget;
import java.util.Optional;

/**
 * A command to automatically align the robot with the best visible AprilTag.
 * The robot will rotate to center the target in the camera's view.
 */
public class AutoAlignCommand extends Command {
  
  private final CANDriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;

  /**
   * Creates a new AutoAlignCommand.
   *
   * @param driveSubsystem The subsystem used for driving the robot.
   * @param visionSubsystem The subsystem used for vision processing.
   */
  public AutoAlignCommand(CANDriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    addRequirements(driveSubsystem, visionSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<BestTarget> bestTarget = visionSubsystem.getBestVisibleTarget();

    if (bestTarget.isPresent()) {
      // If a target is visible, calculate the rotation speed using a P controller.
      // The error is the horizontal offset (tx) of the target.
      double error = bestTarget.get().tx;
      double rotationSpeed = error * Constants.PIDConstants.kP_AUTO_ALIGN;

      // Drive the robot with only rotation, no forward movement.
      driveSubsystem.drive(0, rotationSpeed);
    } else {
      // If no target is visible, stop the robot.
      driveSubsystem.drive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the drive motors when the command finishes.
    driveSubsystem.drive(0, 0);
  }

  // This command never finishes on its own; it's meant to be used with whileTrue().
  @Override
  public boolean isFinished() {
    return false;
  }
}
