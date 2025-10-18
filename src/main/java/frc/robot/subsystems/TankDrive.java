package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.NavXPigeon2;

/**
 * Tank drive using CTRE MotionMagic Velocity.
 * - Leader+Follower on each side
 * - Kinematics + Odometry with navX (wrapped to Pigeon2-like API)
 * - Simulation support and pose logging via DogLog (no Field2d/SmartDashboard).
 */
public class TankDrive extends SubsystemBase {
  // ---- Hardware ----
  private final TalonFX leftLeader    = new TalonFX(Constants.DriveConstants.LEFT_LEADER_ID);
  private final TalonFX leftFollower  = new TalonFX(Constants.DriveConstants.LEFT_FOLLOWER_ID);
  private final TalonFX rightLeader   = new TalonFX(Constants.DriveConstants.RIGHT_LEADER_ID);
  private final TalonFX rightFollower = new TalonFX(Constants.DriveConstants.RIGHT_FOLLOWER_ID);
  private final NavXPigeon2 pigeon    = new NavXPigeon2(); // navX wrapper with Pigeon2-like API

  // ---- Controls ----
  private final MotionMagicVelocityVoltage mmvLeft  = new MotionMagicVelocityVoltage(0).withSlot(0);
  private final MotionMagicVelocityVoltage mmvRight = new MotionMagicVelocityVoltage(0).withSlot(0);

  // ---- Kinematics/Odometry ----
  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(Constants.DriveConstants.TRACK_WIDTH_METERS);
  private final DifferentialDriveOdometry odometry;

  private Pose2d poseMeters = new Pose2d();

  // ---- Simulation state (simple integration) ----
  private double simLeftPosRot = 0.0;   // wheel rotations
  private double simRightPosRot = 0.0;  // wheel rotations
  private double simLeftRps = 0.0;      // wheel RPS
  private double simRightRps = 0.0;     // wheel RPS
  private double simOmegaRadPerSec = 0.0;
  private double simYawRad = 0.0;       // absolute yaw

  public TankDrive() {
    configureDriveTalon(leftLeader,  /*invert=*/false);
    configureDriveTalon(rightLeader, /*invert=*/true); // typical tank: right inverted

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

    odometry = new DifferentialDriveOdometry(getHeadingRotation2d(), 0.0, 0.0, new Pose2d());
  }

  private void configureDriveTalon(TalonFX fx, boolean invert) {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    // Mechanism = wheel rotations (motor -> wheel via gear ratio)
    cfg.Feedback.SensorToMechanismRatio = Constants.DriveConstants.DRIVE_GEARING;

    var s0 = new Slot0Configs();
    s0.kS = Constants.DriveConstants.kS;
    s0.kV = Constants.DriveConstants.kV;
    s0.kA = Constants.DriveConstants.kA;
    s0.kP = Constants.DriveConstants.kP;
    s0.kI = Constants.DriveConstants.kI;
    s0.kD = Constants.DriveConstants.kD;
    cfg.Slot0 = s0;

    var mm = new MotionMagicConfigs();
    mm.MotionMagicAcceleration = mpsToWheelRps(Constants.DriveConstants.ACCEL_MPS2); // RPS^2
    cfg.MotionMagic = mm;

    cfg.MotorOutput.Inverted = invert
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    fx.getConfigurator().apply(cfg);
  }

  /** Arcade drive using normalized inputs [-1..1]. */
  public void arcadeDriveNormalized(double fwd, double rot) {
    double vx = fwd * Constants.DriveConstants.OPEN_LOOP_SCALE * Constants.DriveConstants.CRUISE_MPS;
    double maxOmega = 2.0 * Constants.DriveConstants.CRUISE_MPS / Constants.DriveConstants.TRACK_WIDTH_METERS;
    double wz = rot * maxOmega;
    setChassisSpeeds(new ChassisSpeeds(vx, 0.0, wz));
  }

  /** Directly set desired chassis speeds (m/s, rad/s). */
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    var wheel = kinematics.toWheelSpeeds(speeds);
    wheel.desaturate(Constants.DriveConstants.CRUISE_MPS);

    double leftRps  = mpsToWheelRps(wheel.leftMetersPerSecond);
    double rightRps = mpsToWheelRps(wheel.rightMetersPerSecond);

    leftLeader.setControl(mmvLeft.withVelocity(leftRps));
    rightLeader.setControl(mmvRight.withVelocity(rightRps));

    if (RobotBase.isSimulation()) {
      simLeftRps = leftRps;
      simRightRps = rightRps;
      simOmegaRadPerSec = speeds.omegaRadiansPerSecond;
    }
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

  // ---------------- Telemetry / Odometry ----------------

  public Pose2d getPose() { return poseMeters; }

  public void resetPose(Pose2d newPose) {
    leftLeader.setPosition(0);
    rightLeader.setPosition(0);
    pigeon.setYaw(newPose.getRotation());
    odometry.resetPosition(getHeadingRotation2d(), 0.0, 0.0, newPose);
    poseMeters = newPose;

    if (RobotBase.isSimulation()) {
      simLeftPosRot = 0.0;
      simRightPosRot = 0.0;
      simYawRad = newPose.getRotation().getRadians();
    }
  }

  /** Heading from navX wrapper (CCW+). */
  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
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
  public double getLeftMeters() {
    if (RobotBase.isSimulation()) {
      return simLeftPosRot * Constants.DriveConstants.WHEEL_CIRCUMFERENCE_M;
    }
    double leftRot = leftLeader.getPosition().getValueAsDouble();
    return leftRot * Constants.DriveConstants.WHEEL_CIRCUMFERENCE_M;
  }

  /** Cumulative right wheel travel in meters (from TalonFX position). */
  public double getRightMeters() {
    if (RobotBase.isSimulation()) {
      return simRightPosRot * Constants.DriveConstants.WHEEL_CIRCUMFERENCE_M;
    }
    double rightRot = rightLeader.getPosition().getValueAsDouble();
    return rightRot * Constants.DriveConstants.WHEEL_CIRCUMFERENCE_M;
  }

  @Override
  public void periodic() {
    double leftMeters  = getLeftMeters();
    double rightMeters = getRightMeters();

    poseMeters = odometry.update(getHeadingRotation2d(), leftMeters, rightMeters);

    DogLog.log("Drive/Pose", poseMeters);
  }

  @Override
  public void simulationPeriodic() {
    final double dt = 0.02; // 20ms loop
    simLeftPosRot  += simLeftRps  * dt;
    simRightPosRot += simRightRps * dt;

    simYawRad += simOmegaRadPerSec * dt;
    pigeon.setYaw(Rotation2d.fromRadians(simYawRad));

    leftLeader.setPosition(simLeftPosRot);
    rightLeader.setPosition(simRightPosRot);
  }

  // ---------------- Conversions ----------------

  private static double mpsToWheelRps(double metersPerSecond) {
    return metersPerSecond / Constants.DriveConstants.WHEEL_CIRCUMFERENCE_M;
  }

  private static double wheelRpsToMps(double wheelRps) {
    return wheelRps * Constants.DriveConstants.WHEEL_CIRCUMFERENCE_M;
  }
}
