// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import java.util.Optional;

/**
 * The VisionSubsystem is responsible for interfacing with the Limelight cameras
 * and providing high-level data like the distance to a target and robot pose.
 */
public class VisionSubsystem extends SubsystemBase {

  // Make sure your Limelights are named "limelight-left" and "limelight-right" in their web interfaces.
  private final String[] limelightNames = {"limelight-left", "limelight-right"};
  private final Field2d m_field = new Field2d();

  public VisionSubsystem() {
    // Publish the field widget to Shuffleboard/Elastic
    SmartDashboard.putData("Field", m_field);
  }

  /**
   * Represents the best visible AprilTag target from either Limelight.
   */
  public static class BestTarget {
    public final int id;
    public final double tx; // Horizontal offset
    public final double ty; // Vertical offset
    public final String cameraName;

    public BestTarget(int id, double tx, double ty, String cameraName) {
      this.id = id;
      this.tx = tx;
      this.ty = ty;
      this.cameraName = cameraName;
    }
  }

  /**
   * Finds the best AprilTag target from all available Limelights, but only considers
   * tags that are valid for scoring on the REEF.
   * The "best" target is defined as the one with the largest screen area (ta).
   * @return An Optional containing the BestTarget, or empty if no valid scoring tags are visible.
   */
  public Optional<BestTarget> getBestVisibleTarget() {
    BestTarget bestTarget = null;
    double maxTa = -1.0;

    for (String name : limelightNames) {
      if (LimelightHelpers.getTV(name)) {
        int id = (int) LimelightHelpers.getFiducialID(name);
        
        // This is the crucial check: Is the detected tag in our list of valid scoring tags?
        if (VisionConstants.CORAL_SCORING_TAG_IDS.contains(id)) {
          double ta = LimelightHelpers.getTA(name);
          if (ta > maxTa) {
            maxTa = ta;
            double tx = LimelightHelpers.getTX(name);
            double ty = LimelightHelpers.getTY(name);
            bestTarget = new BestTarget(id, tx, ty, name);
          }
        }
      }
    }
    return Optional.ofNullable(bestTarget);
  }

  /**
   * Calculates the horizontal distance from the robot's camera to the best visible AprilTag.
   * This uses trigonometry based on the specific camera's known mounting angle and height.
   * @return An Optional containing the distance in meters, or empty if no target is visible.
   */
  public Optional<Double> getDistanceToBestTarget() {
    Optional<BestTarget> targetOptional = getBestVisibleTarget();

    if (targetOptional.isEmpty()) {
      return Optional.empty();
    }

    BestTarget bestTarget = targetOptional.get();
    
    // Get the known pose of the AprilTag from our field map
    Pose3d tagPose = FieldConstants.APRIL_TAG_FIELD_LAYOUT.get(bestTarget.id);
    if (tagPose == null) {
      return Optional.empty(); // Tag ID not in our map
    }
    double targetHeight = tagPose.getZ();

    // Dynamically select the correct camera constants based on which one saw the target
    double cameraHeight;
    double cameraPitch;

    if (bestTarget.cameraName.equals("limelight-left")) {
      cameraHeight = VisionConstants.LEFT_CAMERA_HEIGHT_METERS;
      cameraPitch = VisionConstants.LEFT_CAMERA_PITCH_RADIANS;
    } else {
      // Default to right camera if not left
      cameraHeight = VisionConstants.RIGHT_CAMERA_HEIGHT_METERS;
      cameraPitch = VisionConstants.RIGHT_CAMERA_PITCH_RADIANS;
    }

    // The core trigonometric calculation
    // distance = (targetHeight - cameraHeight) / tan(cameraAngle + targetVerticalOffset)
    double distance = (targetHeight - cameraHeight) / 
                      Math.tan(cameraPitch + Units.degreesToRadians(bestTarget.ty));

    return Optional.of(distance);
  }

  /**
   * Calculates the robot's pose on the field using the Limelight's raw camera-to-target transform.
   * @param limelightName The name of the Limelight to use.
   * @return An Optional containing the robot's calculated Pose2d, or empty if no target is visible.
   */
  private Optional<Pose2d> getPoseFromCamtran(String limelightName) {
    if (!LimelightHelpers.getTV(limelightName)) {
        return Optional.empty();
    }

    int tagId = (int) LimelightHelpers.getFiducialID(limelightName);
    Pose3d tagPoseOnField = FieldConstants.APRIL_TAG_FIELD_LAYOUT.get(tagId);

    if (tagPoseOnField == null) {
        return Optional.empty(); // We see a tag, but it's not on our map
    }

    // Get the camera-to-target transform from the Limelight
    double[] camtran = LimelightHelpers.getLimelightNTDoubleArray(limelightName, "camtran");
    if (camtran.length < 6) {
        return Optional.empty();
    }

    // Limelight's camtran is (x, y, z, pitch, yaw, roll) in target space
    // We need to convert this to a standard Transform3d
    // Limelight's coordinate system: +x right, +y down, +z forward
    // WPILib's robot-centric system: +x forward, +y left, +z up
    Transform3d cameraToTarget = new Transform3d(
        new Translation3d(camtran[2], -camtran[0], -camtran[1]), // Z -> X, -X -> Y, -Y -> Z
        new Rotation3d(
            Units.degreesToRadians(-camtran[4]), // -Yaw
            Units.degreesToRadians(-camtran[3]), // -Pitch
            Units.degreesToRadians(camtran[5])   // Roll
        )
    );

    // Get the robot-to-camera transform from Constants
    Transform3d robotToCamera;
    if (limelightName.equals("limelight-left")) {
      robotToCamera = VisionConstants.LEFT_ROBOT_TO_CAMERA;
    } else {
      robotToCamera = VisionConstants.RIGHT_ROBOT_TO_CAMERA;
    }

    // Calculate pose
    Pose3d robotPoseOnField = tagPoseOnField
        .transformBy(cameraToTarget.inverse())
        .transformBy(robotToCamera.inverse());

    // Flip pose if we are on the Red Alliance
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    if (isRed) {
        return Optional.of(
            new Pose2d(
                FieldConstants.FIELD_LENGTH_METERS - robotPoseOnField.getX(),
                FieldConstants.FIELD_WIDTH_METERS - robotPoseOnField.getY(),
                robotPoseOnField.getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))
            )
        );
    }
    
    return Optional.of(robotPoseOnField.toPose2d());
  }

  @Override
  public void periodic() {
    // --- DEBUGGING WIDGETS ---
    for (String name : limelightNames) {
        if(LimelightHelpers.getTV(name)) {
            SmartDashboard.putNumber("RAW Tag ID (" + name + ")", LimelightHelpers.getFiducialID(name));
        } else {
            SmartDashboard.putNumber("RAW Tag ID (" + name + ")", -1);
        }
    }

    // --- MAIN TARGETING LOGIC ---
    Optional<BestTarget> bestTarget = getBestVisibleTarget();

    if (bestTarget.isPresent()) {
      SmartDashboard.putNumber("Targeted AprilTag ID", bestTarget.get().id);
    } else {
      SmartDashboard.putNumber("Targeted AprilTag ID", -999);
    }

    // --- POSE ESTIMATION AND MAP WIDGET ---
    for (String name : limelightNames) {
        Optional<Pose2d> robotPose = getPoseFromCamtran(name);
        if (robotPose.isPresent()) {
            m_field.setRobotPose(robotPose.get());
            // We got a valid pose, no need to check the other camera
            return;
        }
    }
    // If no tags are visible, clear the robot pose from the map
    m_field.setRobotPose(new Pose2d(-1, -1, new Rotation2d()));
  }
}

