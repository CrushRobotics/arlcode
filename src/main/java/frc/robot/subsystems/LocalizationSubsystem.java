package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LocalizationConstants;

import java.util.Map;
import java.util.Optional;

/**
 * The LocalizationSubsystem is responsible for fusing sensor data from the
 * drivetrain and vision system to provide a single, accurate estimate of the
 * robot's position on the field.
 */
public class LocalizationSubsystem extends SubsystemBase {

    private final TankDriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    private final DifferentialDrivePoseEstimator poseEstimator;
    private final Field2d field = new Field2d();

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(LocalizationConstants.TRACK_WIDTH_METERS);

    public LocalizationSubsystem(TankDriveSubsystem drive, VisionSubsystem vision) {
        this.driveSubsystem = drive;
        this.visionSubsystem = vision;

        poseEstimator = new DifferentialDrivePoseEstimator(
                kinematics,
                driveSubsystem.getRotation2d(),
                driveSubsystem.getLeftDistanceMeters(),
                driveSubsystem.getRightDistanceMeters(),
                new Pose2d());

        SmartDashboard.putData("Field", field);

        // Configure PathPlanner logging
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            field.setRobotPose(pose);
        });

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            field.getObject("target pose").setPose(pose);
        });

        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path").setPoses(poses);
        });
    }

    @Override
    public void periodic() {
        poseEstimator.update(
                driveSubsystem.getRotation2d(),
                driveSubsystem.getLeftDistanceMeters(),
                driveSubsystem.getRightDistanceMeters());

        addVisionMeasurement("limelight-right");
        addVisionMeasurement("limelight-left");

        field.setRobotPose(getPose());
    }
    
    private void addVisionMeasurement(String limelightName) {
        PoseEstimate visionPoseEstimate = visionSubsystem.getPoseEstimate(limelightName);

        SmartDashboard.putBoolean("Localization/" + limelightName + "/HasPoseEstimate", visionPoseEstimate != null);

        if (visionPoseEstimate != null && LimelightHelpers.validPoseEstimate(visionPoseEstimate)) {
            // THE FIX for simulation turning:
            // The original `isStationary` check caused a feedback loop in simulation where commanding
            // a turn would prevent vision updates, which in turn prevented the simulated gyro
            // from updating correctly. Removing this check fixes turning in the simulator.
            // On a real robot, you might re-introduce a check to only add vision measurements
            // when tag ambiguity is low or when the robot is moving slowly.
            
            SmartDashboard.putBoolean("Localization/" + limelightName + "/AddingVisionMeasurement", true);
            
            // Add the measurement to the pose estimator.
            poseEstimator.addVisionMeasurement(
                visionPoseEstimate.pose, 
                visionPoseEstimate.timestampSeconds);
        } else {
            SmartDashboard.putBoolean("Localization/" + limelightName + "/AddingVisionMeasurement", false);
        }
    }


    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d newPose) {
        driveSubsystem.resetPose(newPose); // Also reset the drive subsystem's odometry
        poseEstimator.resetPosition(
            driveSubsystem.getRotation2d(),
            driveSubsystem.getLeftDistanceMeters(),
            driveSubsystem.getRightDistanceMeters(),
            newPose);
    }

    public void simulationPeriodic() {
      // --- Simulate Vision Data ---
      Pose2d currentPose = getPose();
      Optional<Pose3d> closestTagPose = Optional.empty();
      int closestTagId = -1;
      double minDistance = Double.MAX_VALUE;

      // Find the closest AprilTag to the robot
      for (Map.Entry<Integer, Pose3d> entry : FieldConstants.APRIL_TAG_FIELD_LAYOUT.entrySet()) {
          double distance = entry.getValue().toPose2d().getTranslation().getDistance(currentPose.getTranslation());
          if (distance < minDistance) {
              minDistance = distance;
              closestTagPose = Optional.of(entry.getValue());
              closestTagId = entry.getKey();
          }
      }

      PoseEstimate estimate = null;
      // If a tag is found and within a reasonable distance...
      if (closestTagPose.isPresent() && minDistance < 5.0) { // Simulate seeing tags within 5 meters
          Pose2d tagPose2d = closestTagPose.get().toPose2d();
          
          // Check if the robot is generally facing the tag (e.g., within a 120-degree FOV)
          double angleToTag = Math.atan2(tagPose2d.getY() - currentPose.getY(), tagPose2d.getX() - currentPose.getX());
          double angleDifference = Math.abs(currentPose.getRotation().getRadians() - angleToTag);
          
          if (Math.abs(angleDifference) < Units.degreesToRadians(60)) {
               // Create a "perfect" pose estimate based on the robot's actual simulated pose
              RawFiducial[] rawFiducials = new RawFiducial[1];
              rawFiducials[0] = new RawFiducial(closestTagId, 0, 0, 0, minDistance, minDistance, 0.1);

              // Get the correct alliance pose from Limelight
              boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
              Pose2d visionPose = isRed 
                ? LimelightHelpers.getBotPose2d_wpiRed("limelight-right")
                : LimelightHelpers.getBotPose2d_wpiBlue("limelight-right");


              estimate = new PoseEstimate(
                  currentPose, // Use the "real" simulated pose for perfect vision data
                  Timer.getFPGATimestamp() - 0.03, // Simulate 30ms latency
                  30.0,
                  1,
                  0,
                  minDistance,
                  0,
                  rawFiducials,
                  true
              );
          }
      }
      
      // Update the vision subsystem with the simulated data (or null if no target is seen)
      visionSubsystem.updateSimulatedVisionData(estimate);
  }
}
