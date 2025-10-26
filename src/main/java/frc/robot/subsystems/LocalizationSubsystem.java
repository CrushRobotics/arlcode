package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;

import dev.doglog.DogLog;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Constants.DriveConstants;

/**
 * The LocalizationSubsystem is responsible for fusing sensor data from the
 * drivetrain and vision system to provide a single, accurate estimate of the
 * robot's position on the field.
 */
public class LocalizationSubsystem extends SubsystemBase {

    private final CANDriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    // For a tank drive, we use a DifferentialDrivePoseEstimator
    private final DifferentialDrivePoseEstimator poseEstimator;
    private final Field2d field = new Field2d();

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH_METERS);

    public LocalizationSubsystem(CANDriveSubsystem drive, VisionSubsystem vision) {
        this.driveSubsystem = drive;
        this.visionSubsystem = vision;

        // The pose estimator requires the kinematics object, the gyro angle,
        // the wheel distances, and an initial pose.
        poseEstimator = new DifferentialDrivePoseEstimator(
                kinematics,
                driveSubsystem.getRotation2d(),
                driveSubsystem.getLeftDistanceMeters(),
                driveSubsystem.getRightDistanceMeters(),
                new Pose2d()); // Start at (0,0)

        SmartDashboard.putData("Field", field);
        DogLog.log("Estimated Pose", getPose());
    }

    @Override
    public void periodic() {
        // Update the pose estimator with the latest drive odometry
        poseEstimator.update(
                driveSubsystem.getRotation2d(),
                driveSubsystem.getLeftDistanceMeters(),
                driveSubsystem.getRightDistanceMeters());

        if (!Utils.isSimulation()) {
            // Fuse vision data from BOTH Limelights
            addVisionMeasurement("limelight-right");
            addVisionMeasurement("limelight-left");
        }

        // Update the Field2d widget with the estimated pose
        field.setRobotPose(getPose());
        DogLog.log("Estimated Pose", getPose());
    }
    
    /**
     * Attempts to get a pose estimate from the specified Limelight and add it
     * to the pose estimator.
     * @param limelightName The network table name of the Limelight.
     */
    private void addVisionMeasurement(String limelightName) {
        PoseEstimate visionPoseEstimate = visionSubsystem.getPoseEstimate(limelightName);

        // DEBUG: Check if we are receiving a pose estimate at all
        SmartDashboard.putBoolean("Localization/" + limelightName + "/HasPoseEstimate", visionPoseEstimate != null);

        if (visionPoseEstimate != null && LimelightHelpers.validPoseEstimate(visionPoseEstimate)) {
             // DEBUG: Confirm that we are entering the block to add the measurement
             SmartDashboard.putBoolean("Localization/" + limelightName + "/AddingVisionMeasurement", true);
             
             // We have a valid pose from the camera. Let's add it to the estimator.
             // We can trust Limelight's timestamp for this.
             poseEstimator.addVisionMeasurement(
                visionPoseEstimate.pose, 
                visionPoseEstimate.timestampSeconds);
            if (Math.abs(driveSubsystem.getChassisSpeeds().vxMetersPerSecond) <  0.1 && Math.abs(driveSubsystem.getChassisSpeeds().omegaRadiansPerSecond) < 0.5) {
                resetOdometryAndGyroToLimelight();
            }
        } else {
            // DEBUG: If we don't add a measurement, explicitly say so.
            SmartDashboard.putBoolean("Localization/" + limelightName + "/AddingVisionMeasurement", false);
        }
    }

    public void resetOdometryAndGyroToLimelight() {
        // System.out.println("Attempting to reset Odometry and Gyro to Limelight...");
        // Prefer right limelight, fallback to left
        LimelightHelpers.PoseEstimate limelightPoseEstimate = visionSubsystem.getPoseEstimate("limelight-right");
        LimelightHelpers.PoseEstimate limelightPoseEstimateMT1 = visionSubsystem.getPoseEstimateMT1("limelight-right");
        String source = "limelight-right";
    
        if (limelightPoseEstimate == null || !LimelightHelpers.validPoseEstimate(limelightPoseEstimate)) {
            limelightPoseEstimate = visionSubsystem.getPoseEstimate("limelight-left");
            source = "limelight-left";
        }
    
        if (limelightPoseEstimate != null && LimelightHelpers.validPoseEstimate(limelightPoseEstimate)) {
            // System.out.println("Valid Pose Estimate found from " + source + ": " + limelightPoseEstimate.pose);
    
            // 1. Reset the Pose Estimator
            resetPose(limelightPoseEstimateMT1.pose);
            // System.out.println("Pose Estimator reset to: " + limelightPoseEstimate.pose);
    
            // 2. Reset the NavX Gyro Yaw
            driveSubsystem.getPigeon().setYaw(limelightPoseEstimateMT1.pose.getRotation());
            // System.out.println("NavX Yaw reset to: " + limelightPoseEstimate.pose.getRotation().getDegrees());
    
        } else {
            System.err.println("Reset Odometry/Gyro to Limelight FAILED: No valid pose estimate found from either camera.");
        }
    }

    /**
     * @return The current estimated pose of the robot.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the robot's pose to a new value. This is useful for telling the
     * robot where it is at the start of a match.
     * @param newPose The new pose for the robot.
     */
    public void resetPose(Pose2d newPose) {
        poseEstimator.resetPosition(
            driveSubsystem.getRotation2d(),
            driveSubsystem.getLeftDistanceMeters(),
            driveSubsystem.getRightDistanceMeters(),
            newPose);
    }
}
