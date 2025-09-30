package frc.robot.subsystems;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Constants.LocalizationConstants;

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

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(LocalizationConstants.TRACK_WIDTH_METERS);

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
    }

    @Override
    public void periodic() {
        // Update the pose estimator with the latest drive odometry
        poseEstimator.update(
                driveSubsystem.getRotation2d(),
                driveSubsystem.getLeftDistanceMeters(),
                driveSubsystem.getRightDistanceMeters());

        // Fuse vision data from both Limelights
        addVisionMeasurement("limelight-left");
        addVisionMeasurement("limelight-right");

        // Update the Field2d widget with the estimated pose
        field.setRobotPose(getPose());
    }
    
    /**
     * Attempts to get a pose estimate from the specified Limelight and add it
     * to the pose estimator.
     * @param limelightName The network table name of the Limelight.
     */
    private void addVisionMeasurement(String limelightName) {
        PoseEstimate visionPoseEstimate = visionSubsystem.getMegaTag2Pose(limelightName);

        if (visionPoseEstimate != null && LimelightHelpers.validPoseEstimate(visionPoseEstimate)) {
             // We have a valid pose from the camera. Let's add it to the estimator.
             // We can trust Limelight's timestamp for this.
             poseEstimator.addVisionMeasurement(
                visionPoseEstimate.pose, 
                visionPoseEstimate.timestampSeconds);
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

