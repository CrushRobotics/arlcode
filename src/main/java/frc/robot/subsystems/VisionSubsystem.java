package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        // Send robot orientation to the Limelight for MegaTag2
        // We get the latest heading from the drive subsystem's NavX
        double heading = driveSubsystem.getHeading();
        LimelightHelpers.SetRobotOrientation("limelight-right", heading, 0, 0, 0, 0, 0);

        // --- DEBUGGING OUTPUTS ---
        SmartDashboard.putNumber("Vision/SentHeading", heading);
        
        // Get the latest pose estimate to display its data
        PoseEstimate poseEstimate = getMegaTag2Pose("limelight-right");

        // Check if the Limelight has any valid target
        boolean hasTarget = LimelightHelpers.getTV("limelight-right");
        SmartDashboard.putBoolean("Vision/HasTarget", hasTarget);

        if (hasTarget && poseEstimate != null && LimelightHelpers.validPoseEstimate(poseEstimate)) {
            // We have a valid pose from the camera
            SmartDashboard.putNumber("Vision/PoseX", poseEstimate.pose.getX());
            SmartDashboard.putNumber("Vision/PoseY", poseEstimate.pose.getY());
            SmartDashboard.putNumber("Vision/PoseRotation", poseEstimate.pose.getRotation().getDegrees());
            SmartDashboard.putNumber("Vision/TagCount", poseEstimate.tagCount);
            SmartDashboard.putNumber("Vision/Latency", poseEstimate.latency);
        } else {
            // No valid target, set default values
            SmartDashboard.putNumber("Vision/TagCount", 0);
        }
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

