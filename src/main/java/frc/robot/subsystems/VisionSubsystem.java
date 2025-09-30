package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

/**
 * The VisionSubsystem is responsible for interfacing with the Limelight cameras
 * and providing pose estimates.
 */
public class VisionSubsystem extends SubsystemBase {
    
    private final CANDriveSubsystem driveSubsystem;

    // We pass in the drive subsystem to get the robot's orientation (from the NavX)
    public VisionSubsystem(CANDriveSubsystem drive) {
        this.driveSubsystem = drive;
    }

    @Override
    public void periodic() {
        // Send robot orientation to both Limelights for MegaTag2
        // We get the latest heading from the drive subsystem's NavX
        double heading = driveSubsystem.getHeading();
        LimelightHelpers.SetRobotOrientation("limelight-left", heading, 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation("limelight-right", heading, 0, 0, 0, 0, 0);
    }

    /**
     * Gets the latest MegaTag2 pose estimate from a specified Limelight.
     * @param limelightName The name of the Limelight camera.
     * @return A PoseEstimate object, which may or may not contain a valid pose.
     */
    public PoseEstimate getMegaTag2Pose(String limelightName) {
        // We use the wpiBlue coordinate system by default.
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    }

    /**
     * Gets the 2D pose from a specific limelight.
     * @param limelightName The name of the limelight.
     * @return A Pose2d, or null if no target is found.
     */
    public Pose2d getPose2d(String limelightName) {
        return LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
    }
}
