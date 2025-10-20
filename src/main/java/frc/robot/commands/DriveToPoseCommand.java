package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.Constants.AutoConstants;

/**
 * A command to drive the robot to a specific pose on the field using PID controllers.
 */
public class DriveToPoseCommand extends Command {

    private final CANDriveSubsystem driveSubsystem;
    private final LocalizationSubsystem localizationSubsystem;
    private final Pose2d targetPose;

    private final PIDController driveController;
    private final PIDController turnController;

    /**
     * Creates a new DriveToPoseCommand.
     * @param drive The drive subsystem.
     * @param localization The localization subsystem.
     * @param target The target pose to drive to.
     */
    public DriveToPoseCommand(CANDriveSubsystem drive, LocalizationSubsystem localization, Pose2d target) {
        this.driveSubsystem = drive;
        this.localizationSubsystem = localization;
        this.targetPose = target;

        // Initialize PID controllers with constants
        driveController = new PIDController(AutoConstants.kP_DRIVE_TO_POSE, 0, 0);
        turnController = new PIDController(AutoConstants.kP_TURN_TO_POSE, 0, 0);
        
        // Allow the turn controller to handle wrapping from -180 to 180 degrees
        turnController.enableContinuousInput(-180, 180);

        // Set tolerances for the PID controllers to use with atSetpoint()
        driveController.setTolerance(AutoConstants.DRIVE_TO_POSE_TOLERANCE_METERS);
        turnController.setTolerance(AutoConstants.TURN_TO_POSE_TOLERANCE_DEGREES);

        addRequirements(drive, localization);
    }

    @Override
    public void execute() {
        Pose2d currentPose = localizationSubsystem.getPose();
        
        // Calculate the vector from the robot to the target
        Translation2d translationToTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
        
        // The desired heading is to face the target location
        Rotation2d desiredRotation = translationToTarget.getAngle();

        // Use PID controllers to calculate drive and turn speeds simultaneously
        double driveSpeed = -driveController.calculate(translationToTarget.getNorm(), 0);
        double turnSpeed = turnController.calculate(currentPose.getRotation().getDegrees(), desiredRotation.getDegrees());

        // Clamp the speeds to a reasonable maximum
        driveSpeed = Math.max(-0.6, Math.min(0.6, driveSpeed));
        turnSpeed = Math.max(-0.6, Math.min(0.6, turnSpeed));

        driveSubsystem.arcadeDrive(driveSpeed, turnSpeed);
    }

    @Override
    public boolean isFinished() {
        // The command is finished when the robot is at the target location AND facing
        // the correct direction. We use the PID controller's atSetpoint() method
        // which considers the tolerance we set in the constructor.
        return driveController.atSetpoint() && turnController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drive motors when the command ends
        driveSubsystem.stop();
    }
}
