package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LocalizationConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.util.NavXPigeon2;

/**
 * A tank drive subsystem that uses CTRE's Motion Magic for velocity control.
 * This is based on the suggestion document to enable robust trajectory following.
 */
public class TankDriveSubsystem extends SubsystemBase {
  // ---- Hardware ----
  private final TalonFX leftLeader    = new TalonFX(DriveConstants.LEFT_LEADER_ID);
  private final TalonFX leftFollower  = new TalonFX(DriveConstants.LEFT_FOLLOWER_ID);
  private final TalonFX rightLeader   = new TalonFX(DriveConstants.RIGHT_LEADER_ID);
  private final TalonFX rightFollower = new TalonFX(DriveConstants.RIGHT_FOLLOWER_ID);
  private final NavXPigeon2 pigeon    = new NavXPigeon2(); // navX wrapper with Pigeon2-like API

  // ---- Controls ----
  private final MotionMagicVelocityVoltage mmvLeft  = new MotionMagicVelocityVoltage(0).withSlot(0);
  private final MotionMagicVelocityVoltage mmvRight = new MotionMagicVelocityVoltage(0).withSlot(0);

  // ---- Kinematics/Odometry ----
  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(LocalizationConstants.TRACK_WIDTH_METERS);
  private final DifferentialDriveOdometry odometry;

  private Pose2d poseMeters = new Pose2d();

  // ---- Simulation state (simple integration) ----
  private double simLeftPosRot = 0.0;   // wheel rotations
  private double simRightPosRot = 0.0;  // wheel rotations
  private double simLeftRps = 0.0;      // wheel RPS
  private double simRightRps = 0.0;     // wheel RPS
  private double simOmegaRadPerSec = 0.0;
  private double simYawRad = 0.0;       // absolute yaw

  public TankDriveSubsystem() {
    configureDriveTalon(leftLeader,  /*invert=*/false);
    configureDriveTalon(rightLeader, /*invert=*/true); // Right side is inverted on a tank drive

    leftFollower.setControl(new Follower(leftLeader.getDeviceID(), false));
    rightFollower.setControl(new Follower(rightLeader.getDeviceID(), false));

    leftLeader.setNeutralMode(NeutralModeValue.Brake);
    leftFollower.setNeutralMode(NeutralModeValue.Brake);
    rightLeader.setNeutralMode(NeutralModeValue.Brake);
    rightFollower.setNeutralMode(NeutralModeValue.Brake);

    leftLeader.setPosition(0);
    rightLeader.setPosition(0);
    pigeon.reset();
    pigeon.setYaw(0.0);

    odometry = new DifferentialDriveOdometry(getRotation2d(), 0.0, 0.0, new Pose2d());
  }

  private void configureDriveTalon(TalonFX fx, boolean invert) {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    // The SensorToMechanismRatio is the gearing from the motor rotations to the wheel rotations
    cfg.Feedback.SensorToMechanismRatio = DriveConstants.DRIVE_GEARING;

    // Configure the PID and feedforward gains for velocity control
    var slot0 = new Slot0Configs();
    slot0.kS = TrajectoryConstants.kS;
    slot0.kV = TrajectoryConstants.kV;
    slot0.kA = TrajectoryConstants.kA;
    slot0.kP = TrajectoryConstants.kP;
    slot0.kI = TrajectoryConstants.kI;
    slot0.kD = TrajectoryConstants.kD;
    cfg.Slot0 = slot0;

    // Configure Motion Magic acceleration
    var motionMagic = new MotionMagicConfigs();
    motionMagic.MotionMagicAcceleration = mpsToWheelRps(TrajectoryConstants.ACCEL_MPS2); // RPS^2
    cfg.MotionMagic = motionMagic;

    cfg.MotorOutput.Inverted = invert
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    fx.getConfigurator().apply(cfg);
  }

  /** Arcade drive using normalized inputs [-1..1] for teleop. */
  public void arcadeDriveNormalized(double fwd, double rot) {
    // Scale inputs to physical units (m/s and rad/s)
    double vx = fwd * TrajectoryConstants.CRUISE_MPS;
    double maxOmega = 2.0 * TrajectoryConstants.CRUISE_MPS / LocalizationConstants.TRACK_WIDTH_METERS;
    double wz = rot * maxOmega;
    setChassisSpeeds(new ChassisSpeeds(vx, 0.0, wz));
  }

  /** Directly set desired chassis speeds (m/s, rad/s). Used for trajectory following. */
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    wheelSpeeds.desaturate(TrajectoryConstants.CRUISE_MPS);

    double leftRps  = mpsToWheelRps(wheelSpeeds.leftMetersPerSecond);
    double rightRps = mpsToWheelRps(wheelSpeeds.rightMetersPerSecond);

    leftLeader.setControl(mmvLeft.withVelocity(leftRps));
    rightLeader.setControl(mmvRight.withVelocity(rightRps));

    if (RobotBase.isSimulation()) {
      simLeftRps = leftRps;
      simRightRps = rightRps;
      simOmegaRadPerSec = speeds.omegaRadiansPerSecond;
    }
  }
  
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  public void stop() {
    leftLeader.setControl(mmvLeft.withVelocity(0));
    rightLeader.setControl(mmvRight.withVelocity(0));
    if (RobotBase.isSimulation()) {
      simLeftRps = 0.0;
      simRightRps = 0.0;
      simOmegaRadPerSec = 0.0;
    }
  }

  public Pose2d getPose() { return poseMeters; }

  public void resetPose(Pose2d newPose) {
    leftLeader.setPosition(0);
    rightLeader.setPosition(0);
    pigeon.setYaw(newPose.getRotation());
    odometry.resetPosition(getRotation2d(), 0.0, 0.0, newPose);
    poseMeters = newPose;

    if (RobotBase.isSimulation()) {
      simLeftPosRot = 0.0;
      simRightPosRot = 0.0;
      simYawRad = newPose.getRotation().getRadians();
    }
  }

  /** Heading from navX wrapper (CCW+). */
  public Rotation2d getRotation2d() {
    return pigeon.getRotation2d();
  }

  /** Current wheel speeds from TalonFX velocity signals (converted to m/s). */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    if (RobotBase.isSimulation()) {
      return new DifferentialDriveWheelSpeeds(
          wheelRpsToMps(simLeftRps), wheelRpsToMps(simRightRps));
    }
    double leftRps  = leftLeader.getVelocity().getValueAsDouble();
    double rightRps = rightLeader.getVelocity().getValueAsDouble();
    return new DifferentialDriveWheelSpeeds(
        wheelRpsToMps(leftRps), wheelRpsToMps(rightRps));
  }

  /** Cumulative left wheel travel in meters (from TalonFX position). */
  public double getLeftDistanceMeters() {
    if (RobotBase.isSimulation()) {
      return simLeftPosRot * (2 * Math.PI * DriveConstants.WHEEL_RADIUS_METERS);
    }
    // Position signal is in rotations of the wheel
    double leftRot = leftLeader.getPosition().getValueAsDouble();
    return leftRot * (2 * Math.PI * DriveConstants.WHEEL_RADIUS_METERS);
  }

  /** Cumulative right wheel travel in meters (from TalonFX position). */
  public double getRightDistanceMeters() {
    if (RobotBase.isSimulation()) {
      return simRightPosRot * (2 * Math.PI * DriveConstants.WHEEL_RADIUS_METERS);
    }
     // Position signal is in rotations of the wheel
    double rightRot = rightLeader.getPosition().getValueAsDouble();
    return rightRot * (2 * Math.PI * DriveConstants.WHEEL_RADIUS_METERS);
  }

  public double getHeading() {
      return getRotation2d().getDegrees();
  }
  
  public double getForwardVelocityMetersPerSec() {
      return (getWheelSpeeds().leftMetersPerSecond + getWheelSpeeds().rightMetersPerSecond) / 2.0;
  }

  @Override
  public void periodic() {
    double leftMeters  = getLeftDistanceMeters();
    double rightMeters = getRightDistanceMeters();

    poseMeters = odometry.update(getRotation2d(), leftMeters, rightMeters);

    SmartDashboard.putNumber("Drive/PoseX", poseMeters.getX());
    SmartDashboard.putNumber("Drive/PoseY", poseMeters.getY());
    SmartDashboard.putNumber("Drive/PoseDeg", poseMeters.getRotation().getDegrees());
  }

  @Override
  public void simulationPeriodic() {
    final double dt = 0.02; // 20ms loop
    simLeftPosRot  += simLeftRps  * dt;
    simRightPosRot += simRightRps * dt;

    simYawRad += simOmegaRadPerSec * dt;
    pigeon.setYaw(Rotation2d.fromRadians(simYawRad));

    // Update the simulation state of the TalonFXs
    leftLeader.getSimState().setRawRotorPosition(simLeftPosRot * DriveConstants.DRIVE_GEARING);
    leftLeader.getSimState().setRotorVelocity(simLeftRps * DriveConstants.DRIVE_GEARING);
    rightLeader.getSimState().setRawRotorPosition(simRightPosRot * DriveConstants.DRIVE_GEARING);
    rightLeader.getSimState().setRotorVelocity(simRightRps * DriveConstants.DRIVE_GEARING);
  }

  // ---- Conversions ----
  private static double mpsToWheelRps(double metersPerSecond) {
    return metersPerSecond / (2 * Math.PI * DriveConstants.WHEEL_RADIUS_METERS);
  }

  private static double wheelRpsToMps(double wheelRps) {
    return wheelRps * (2 * Math.PI * DriveConstants.WHEEL_RADIUS_METERS);
  }
}
//renamed