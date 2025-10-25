package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.DriveConstants;
import frc.robot.util.NavXPigeon2;

public class CANDriveSubsystem extends SubsystemBase {

    private final TalonFX leftLeader = new TalonFX(DriveConstants.LEFT_LEADER_ID);
    private final TalonFX leftFollower = new TalonFX(DriveConstants.LEFT_FOLLOWER_ID);
    private final TalonFX rightLeader = new TalonFX(DriveConstants.RIGHT_LEADER_ID);
    private final TalonFX rightFollower = new TalonFX(DriveConstants.RIGHT_FOLLOWER_ID);

    private final NavXPigeon2 pigeon = new NavXPigeon2();

    private final DifferentialDriveKinematics kinematics =
            new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH_METERS);

    private double leftPositionMeters = 0;
    private double rightPositionMeters = 0;

    private double leftVelocity = 0;
    private double rightVelocity = 0;

    // --- CONTROLS ---
    private final MotionMagicVelocityVoltage mmvLeft = new MotionMagicVelocityVoltage(0).withSlot(0);
    private final MotionMagicVelocityVoltage mmvRight = new MotionMagicVelocityVoltage(0).withSlot(0);

    private LocalizationSubsystem localizationSubsystem;

    public CANDriveSubsystem() {
        // odometry = new DifferentialDriveOdometry(Rotation2d.kZero, 0.0, 0.0, new Pose2d());

        configureDriveTalon(leftLeader, false);   // Left side inverted mechanically?
        configureDriveTalon(rightLeader, true); // Right side not inverted
                // Configure orientation so positive rotor aligns with your configured inversion
        leftLeader.getSimState().Orientation = ChassisReference.Clockwise_Positive;
        rightLeader.getSimState().Orientation = ChassisReference.Clockwise_Positive;

        // Followers mirror their leaders
        leftFollower.setControl(new Follower(leftLeader.getDeviceID(), false));
        rightFollower.setControl(new Follower(rightLeader.getDeviceID(), false));

        // Neutral mode
        leftLeader.setNeutralMode(NeutralModeValue.Brake);
        leftFollower.setNeutralMode(NeutralModeValue.Brake);
        rightLeader.setNeutralMode(NeutralModeValue.Brake);
        rightFollower.setNeutralMode(NeutralModeValue.Brake);
    }

    private void configureDriveTalon(TalonFX fx, boolean invert) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        // Mechanism units = wheel rotations (sensor is rotor) via ratio
        cfg.Feedback.SensorToMechanismRatio = DriveConstants.DRIVE_GEARING / DriveConstants.WHEEL_CIRCUMFERENCE_METERS;

        // Velocity loop gains (in mechanism-units/s)
        var slot0 = new Slot0Configs();
        slot0.kS = DriveConstants.kS;
        slot0.kV = DriveConstants.kV;
        slot0.kA = DriveConstants.kA;
        slot0.kP = DriveConstants.kP_VELOCITY;
        slot0.kI = DriveConstants.kI_VELOCITY;
        slot0.kD = DriveConstants.kD_VELOCITY;
        cfg.Slot0 = slot0;

        // Motion Magic accel in mechanism RPS^2
        var mm = new MotionMagicConfigs();
        mm.MotionMagicCruiseVelocity = DriveConstants.MAX_SPEED_MPS;
        mm.MotionMagicAcceleration = DriveConstants.MAX_ACCELERATION_MPS_SQ;
        cfg.MotionMagic = mm;

        cfg.MotorOutput.Inverted = invert
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        fx.getConfigurator().apply(cfg);
    }

    /**
     * Directly sets voltage (used by SysId).
     */
    public void setVoltage(double leftVolts, double rightVolts) {
        leftLeader.setControl(new VoltageOut(leftVolts));
        rightLeader.setControl(new VoltageOut(rightVolts));
    }

    public void setLocalizationSubsystem(LocalizationSubsystem localizationSubsystem) {
        this.localizationSubsystem = localizationSubsystem;
    }

    public LocalizationSubsystem getLocalizationSubsystem() {
        return localizationSubsystem;
    }

    /** Arcade drive in normalized [-1, 1] inputs. */
    public void arcadeDrive(double metersPerSecond, double radiansPerSec) {
        double vx = metersPerSecond;
        double wz = radiansPerSec;
        setChassisSpeedsTeleop(new ChassisSpeeds(vx, 0.0, wz));
    }
    public void setChassisSpeedsTeleop(ChassisSpeeds speeds) {
        // var newSpeeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        // wheelSpeeds.desaturate(DriveConstants.MAX_SPEED_MPS);

        leftLeader.setControl(mmvLeft.withVelocity(wheelSpeeds.leftMetersPerSecond));
        rightLeader.setControl(mmvRight.withVelocity(wheelSpeeds.rightMetersPerSecond));
    }

    /** Drives using field units. */
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        // var newSpeeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        // wheelSpeeds.desaturate(DriveConstants.MAX_SPEED_MPS);

        leftLeader.setControl(mmvLeft.withVelocity(wheelSpeeds.leftMetersPerSecond));
        rightLeader.setControl(mmvRight.withVelocity(wheelSpeeds.rightMetersPerSecond));
    }

    public void stop() {
        leftLeader.setControl(mmvLeft.withVelocity(0));
        rightLeader.setControl(mmvRight.withVelocity(0));
    }

    public void resetEncoders() {
        leftLeader.setPosition(0);
        rightLeader.setPosition(0);
    }

    public NavXPigeon2 getPigeon() {
        return pigeon;
    }

    public double getLeftDistanceMeters() {
        // Position is in MECHANISM rotations because SensorToMechanismRatio was applied
        return leftPositionMeters;
    }

    public double getRightDistanceMeters() {
        // Position is in MECHANISM rotations because SensorToMechanismRatio was applied
        return rightPositionMeters;
    }

    public Rotation2d getRotation2d() {
        return RobotBase.isReal()
        ? pigeon.getRotation2d()
        : (Rotation2d.fromRadians(-(leftPositionMeters - rightPositionMeters) / DriveConstants.TRACK_WIDTH_METERS));
        // Use the gyro angle for odometry (sim updates it below)
        // return pigeon.getRotation2d();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftVelocity, rightVelocity);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getWheelSpeeds());
    }

    public TalonFX getLeftLeader() {
        return leftLeader;
    }

    public TalonFX getRightLeader() {
        return rightLeader;
    }

    @Override
    public void periodic() {
        leftPositionMeters = leftLeader.getPosition().getValueAsDouble();
        rightPositionMeters = rightLeader.getPosition().getValueAsDouble();
        leftVelocity = leftLeader.getVelocity().getValueAsDouble();
        rightVelocity = rightLeader.getVelocity().getValueAsDouble();
        
        // Rotation2d heading = getRotation2d();
        
        // poseMeters = odometry.update(heading, leftPositionMeters, rightPositionMeters);
        
        DogLog.log("Drive/Left MPS", leftVelocity);
        DogLog.log("Drive/Right MPS", rightVelocity);
        DogLog.log("Drive/Left Meters", leftPositionMeters);
        DogLog.log("Drive/Right Meters", rightPositionMeters);
    }
    private final DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
        DCMotor.getKrakenX60Foc(2), // 2 Kraken X60 on each side of the drivetrain.
        DriveConstants.DRIVE_GEARING, // Standard AndyMark Gearing reduction.
        4.1, // MOI of 2.1 kg m^2 (from CAD model).
        50.0, // Mass of the robot is 26.5 kg.
        DriveConstants.WHEEL_RADIUS_METERS, // Robot uses 3" radius (6" diameter) wheels.
        0.60, // Distance between wheels is _ meters.
        null // VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
    );

    @Override
    public void simulationPeriodic() {
        // 20 ms nominal
        var leftSim = leftLeader.getSimState();
        var rightSim = rightLeader.getSimState();

        // Set supply voltage (battery)
        leftSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        rightSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        m_driveSim.setInputs(
            leftSim.getMotorVoltage(),
            rightSim.getMotorVoltage()
        );

        m_driveSim.update(0.02);
        final var leftPos = m_driveSim.getLeftPositionMeters() * DriveConstants.DRIVE_GEARING / DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
        // This is OK, since the time base is the same
        final var leftVel = m_driveSim.getLeftVelocityMetersPerSecond() * DriveConstants.DRIVE_GEARING / DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
        final var rightPos = m_driveSim.getRightPositionMeters() * DriveConstants.DRIVE_GEARING / DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
        // This is OK, since the time base is the same
        final var rightVel = m_driveSim.getRightVelocityMetersPerSecond() * DriveConstants.DRIVE_GEARING / DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
        // Integrate mechanism position using the reference (simple plant-less sim)
        leftSim.setRawRotorPosition(leftPos);
        leftSim.setRotorVelocity(leftVel);
        rightSim.setRawRotorPosition(rightPos);
        rightSim.setRotorVelocity(rightVel);
    }

    /** SysId logging helper. */
    public void logSysId(SysIdRoutineLog log) {
        // log.motor("drive-left")
        //     .voltage(Units.Volts.of(leftLeader.getMotorVoltage().getValueAsDouble()))
        //     .angularPosition(Units.Rotations.of(leftLeader.getPosition().getValueAsDouble()))
        //     .angularVelocity(Units.RotationsPerSecond.of(leftLeader.getVelocity().getValueAsDouble()));
        // log.motor("drive-right")
        //     .voltage(Units.Volts.of(rightLeader.getMotorVoltage().getValueAsDouble()))
        //     .angularPosition(Units.Rotations.of(rightLeader.getPosition().getValueAsDouble()))
        //     .angularVelocity(Units.RotationsPerSecond.of(rightLeader.getVelocity().getValueAsDouble()));
    }

    public boolean isNear(Pose2d other, double linearTolerance, double angularTolerance) {
        var currentPose = getLocalizationSubsystem().getPose();
        return MathUtil.isNear(currentPose.getX(), other.getX(), linearTolerance)
            && MathUtil.isNear(currentPose.getY(), other.getY(), linearTolerance)
            && MathUtil.isNear(
                currentPose.getRotation().getDegrees(),
                other.getRotation().getDegrees(),
                angularTolerance, -180, 180);
    }
}
