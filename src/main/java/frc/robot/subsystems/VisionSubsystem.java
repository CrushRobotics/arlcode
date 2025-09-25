// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
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
        
        // Only consider tags that are in our list of valid scoring tags
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

  @Override
  public void periodic() {
    // Get the best visible target
    Optional<BestTarget> bestTarget = getBestVisibleTarget();

    // Send the ID of the targeted AprilTag to the dashboard for driver feedback
    if (bestTarget.isPresent()) {
      SmartDashboard.putNumber("Targeted AprilTag ID", bestTarget.get().id);
    } else {
      // If no target is visible, display a default value like -1
      SmartDashboard.putNumber("Targeted AprilTag ID", -999);
    }

    // Update the field map widget with the robot's pose from either Limelight
    for (String name : limelightNames) {
        if(LimelightHelpers.getTV(name)) {
            // Get the Blue Alliance pose from the Limelight
            Pose2d llPose = LimelightHelpers.getBotPose2d_wpiBlue(name);

            // Update the widget if the pose is valid (not all zeros)
            if (llPose.getX() != 0 || llPose.getY() != 0) {
                 m_field.setRobotPose(llPose);
                 // We got a valid pose, no need to check the other camera
                 return;
            }
        }
    }
  }
}
