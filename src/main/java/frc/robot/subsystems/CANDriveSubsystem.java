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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Units; // Ensures Units class is imported
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Import the correct SysId logger class
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// Removed incorrect import
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.MechanismLogger;


import frc.robot.Constants.DriveConstants;
import frc.robot.util.NavXPigeon2;

public class CANDriveSubsystem extends SubsystemBase {

    private final TalonFX leftLeader = new TalonFX(DriveConstants.LEFT_LEADER_ID);
    private final TalonFX leftFollower = new TalonFX(DriveConstants.LEFT_FOLLOWER_ID);
    private final TalonFX rightLeader = new TalonFX(DriveConstants.RIGHT_LEADER_ID);
    private final TalonFX rightFollower = new TalonFX(DriveConstants.RIGHT_FOLLOWER_ID);

    private final NavXPigeon2 pigeon = new NavXPigeon2();

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH_METERS);

    // --- CONTROLS ---
    private final MotionMagicVelocityVoltage mmvLeft = new MotionMagicVelocityVoltage(0).withSlot(0);
    private final MotionMagicVelocityVoltage mmvRight = new MotionMagicVelocityVoltage(0).withSlot(0);

    // --- SIMULATION ---
    private double simLeftPosRot = 0.0;
    private double simRightPosRot = 0.0;
    private double simYawRad = 0.0;
    private ChassisSpeeds simChassisSpeeds = new ChassisSpeeds();


    public CANDriveSubsystem() {
        configureDriveTalon(leftLeader, true); // Left side is inverted mechanically? Assume true based on original code.
        configureDriveTalon(rightLeader, false); // Right side not inverted

        // Followers mirror their leaders
        leftFollower.setControl(new Follower(leftLeader.getDeviceID(), false));
        rightFollower.setControl(new Follower(rightLeader.getDeviceID(), false));

        // Set neutral mode
        leftLeader.setNeutralMode(NeutralModeValue.Brake);
        leftFollower.setNeutralMode(NeutralModeValue.Brake);
        rightLeader.setNeutralMode(NeutralModeValue.Brake);
        rightFollower.setNeutralMode(NeutralModeValue.Brake);
    }

    private void configureDriveTalon(TalonFX fx, boolean invert) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        // Set the gearing ratio for accurate distance measurements
        cfg.Feedback.SensorToMechanismRatio = DriveConstants.DRIVE_GEARING;

        // Configure PID and feedforward gains for velocity control
        var slot0 = new Slot0Configs();
        slot0.kS = DriveConstants.kS;
        slot0.kV = DriveConstants.kV;
        slot0.kA = DriveConstants.kA;
        slot0.kP = DriveConstants.kP_VELOCITY;
        slot0.kI = DriveConstants.kI_VELOCITY;
        slot0.kD = DriveConstants.kD_VELOCITY;
        cfg.Slot0 = slot0;

        // Configure Motion Magic settings
        var mm = new MotionMagicConfigs();
        mm.MotionMagicAcceleration = mpsToWheelRps(DriveConstants.MAX_ACCELERATION_MPS_SQ); // RPS^2
        cfg.MotionMagic = mm;

        // Set motor inversion based on the invert parameter
        cfg.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        fx.getConfigurator().apply(cfg);
    }

    /**
     * Sets the voltage of the drivetrain motors directly.
     * This is used by the SysId routine.
     * @param leftVolts The voltage to apply to the left side motors.
     * @param rightVolts The voltage to apply to the right side motors.
     */
    public void setVoltage(double leftVolts, double rightVolts) {
        leftLeader.setControl(new VoltageOut(leftVolts));
        rightLeader.setControl(new VoltageOut(rightVolts));
    }

    private LocalizationSubsystem localizationSubsystem;

    public void setLocalizationSubsystem(LocalizationSubsystem localizationSubsystem) {
        this.localizationSubsystem = localizationSubsystem;
    }

    public LocalizationSubsystem getLocalizationSubsystem() {
        return localizationSubsystem;
    }

    /**
     * Drives the robot using arcade controls.
     * @param fwd Forward speed (-1.0 to 1.0). Positive is forward.
     * @param rot Rotation speed (-1.0 to 1.0). Positive is counter-clockwise.
     */
    public void arcadeDrive(double fwd, double rot) {
        // Scale inputs to physical units (m/s and rad/s)
        double vx = fwd * DriveConstants.MAX_SPEED_MPS;
        // Use MAX_ANGULAR_SPEED_RAD_PER_SEC from Constants
        double wz = rot * DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC;
        // Create ChassisSpeeds with positive X as forward, positive omega as CCW
        setChassisSpeeds(new ChassisSpeeds(vx, 0.0, wz));
    }

    /**
     * Sets the robot's speed using ChassisSpeeds.
     * @param speeds The desired chassis speeds (vx, vy, omega).
     */
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        // Store the desired speeds for simulation
        if (RobotBase.isSimulation()) {
            this.simChassisSpeeds = speeds;
        }

        // Convert chassis speeds to wheel speeds
        // WPILib convention: +vx = forward, +omega = CCW rotation
        var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        wheelSpeeds.desaturate(DriveConstants.MAX_SPEED_MPS);

        // Convert m/s to wheel rotations per second
        double leftRps  = mpsToWheelRps(wheelSpeeds.leftMetersPerSecond);
        double rightRps = mpsToWheelRps(wheelSpeeds.rightMetersPerSecond);

        // Command the motors using Motion Magic Velocity control
        leftLeader.setControl(mmvLeft.withVelocity(leftRps));
        rightLeader.setControl(mmvRight.withVelocity(rightRps));
    }

    public void stop() {
        leftLeader.setControl(mmvLeft.withVelocity(0));
        rightLeader.setControl(mmvRight.withVelocity(0));
        if (RobotBase.isSimulation()) {
          this.simChassisSpeeds = new ChassisSpeeds();
        }
    }

    public void resetEncoders() {
        leftLeader.setPosition(0);
        rightLeader.setPosition(0);
    }

    public NavXPigeon2 getPigeon() {
        return pigeon;
    }

    public double getLeftDistanceMeters() {
        if (RobotBase.isSimulation()) {
            return simLeftPosRot * DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
        }
        // Position comes from TalonFX in rotations, convert to meters
        return leftLeader.getPosition().getValueAsDouble() * DriveConstants.ROTATIONS_TO_METERS;
    }

    public double getRightDistanceMeters() {
        if (RobotBase.isSimulation()) {
            return simRightPosRot * DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
        }
        // Position comes from TalonFX in rotations, convert to meters
        return rightLeader.getPosition().getValueAsDouble() * DriveConstants.ROTATIONS_TO_METERS;
    }

    public Rotation2d getRotation2d() {
        return pigeon.getRotation2d();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        // Velocity comes from TalonFX in RPS, convert to m/s
        double leftRps  = leftLeader.getVelocity().getValueAsDouble();
        double rightRps = rightLeader.getVelocity().getValueAsDouble();
        return new DifferentialDriveWheelSpeeds(
            wheelRpsToMps(leftRps), wheelRpsToMps(rightRps));
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
        SmartDashboard.putNumber("Left Drive Distance (m)", getLeftDistanceMeters());
        SmartDashboard.putNumber("Right Drive Distance (m)", getRightDistanceMeters());
        SmartDashboard.putNumber("Robot Heading (deg)", getRotation2d().getDegrees());
        SmartDashboard.putNumber("Left Drive Velocity (RPS)", leftLeader.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Right Drive Velocity (RPS)", rightLeader.getVelocity().getValueAsDouble());
        // Fixed voltage calculation using getValueAsDouble()
        SmartDashboard.putNumber("Left Drive Output (%)", leftLeader.getMotorVoltage().getValueAsDouble() / 12.0); // Approx output percent
        SmartDashboard.putNumber("Right Drive Output (%)", rightLeader.getMotorVoltage().getValueAsDouble() / 12.0); // Approx output percent
    }

    @Override
    public void simulationPeriodic() {
        final double dt = 0.02; // 20ms loop

        // Get target wheel speeds from the last commanded chassis speeds
        // simChassisSpeeds now correctly represents desired robot motion (+vx = forward, +omega = CCW)
        var wheelSpeeds = kinematics.toWheelSpeeds(simChassisSpeeds);
        double leftRps = mpsToWheelRps(wheelSpeeds.leftMetersPerSecond);
        double rightRps = mpsToWheelRps(wheelSpeeds.rightMetersPerSecond);

        // Update simulated positions (based on wheel rotations)
        simLeftPosRot += leftRps * dt;
        simRightPosRot += rightRps * dt;

        // Update simulated heading based on chassis omega
        simYawRad += simChassisSpeeds.omegaRadiansPerSecond * dt;
        pigeon.setYaw(Rotation2d.fromRadians(simYawRad));

        // Update the TalonFX sim states with new data
        // The position and velocity should be in MOTOR rotations (apply gearing)
        leftLeader.getSimState().setRawRotorPosition(simLeftPosRot * DriveConstants.DRIVE_GEARING);
        leftLeader.getSimState().setRotorVelocity(leftRps * DriveConstants.DRIVE_GEARING);

        rightLeader.getSimState().setRawRotorPosition(simRightPosRot * DriveConstants.DRIVE_GEARING);
        rightLeader.getSimState().setRotorVelocity(rightRps * DriveConstants.DRIVE_GEARING);

        // Apply voltage based on velocity for simulation feedback (simple model)
        leftLeader.getSimState().setSupplyVoltage(12.0);
        rightLeader.getSimState().setSupplyVoltage(12.0);
        // Note: For more accurate simulation, a physics model (like WPILib's DifferentialDrivetrainSim)
        // should be used to calculate actual velocities based on applied voltage/control effort.
        // This current implementation directly sets the simulated velocity to the target velocity.
    }

    // --- CONVERSIONS ---
    private static double mpsToWheelRps(double metersPerSecond) {
        return metersPerSecond / DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
    }

    private static double wheelRpsToMps(double wheelRps) {
        // Fixed typo: WHEEL_CIRCUMFERENCE_BETERS -> WHEEL_CIRCUMFERENCE_METERS
        return wheelRps * DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
    }

    /**
     * Method needed for SysId logging.
     * @param log The SysId routine logger.
     */
     // Corrected type to SysIdRoutineLog
    public void logSysId(SysIdRoutineLog log) {
        log.motor("drive-left")
            .voltage(Units.Volts.of(leftLeader.getMotorVoltage().getValueAsDouble())) // Use Units
            // Log position in WHEEL rotations, SysId accounts for gearing if characterization type requires it
            .angularPosition(Units.Rotations.of(leftLeader.getPosition().getValueAsDouble())) // Use Units
            // Log velocity in WHEEL rotations per second
            .angularVelocity(Units.RotationsPerSecond.of(leftLeader.getVelocity().getValueAsDouble())); // Use Units
        log.motor("drive-right")
            .voltage(Units.Volts.of(rightLeader.getMotorVoltage().getValueAsDouble())) // Use Units
            // Log position in WHEEL rotations
            .angularPosition(Units.Rotations.of(rightLeader.getPosition().getValueAsDouble())) // Use Units
            // Log velocity in WHEEL rotations per second
            .angularVelocity(Units.RotationsPerSecond.of(rightLeader.getVelocity().getValueAsDouble())); // Use Units
    }
}

