package frc.robot.subsystems;

// --- CTRE Imports ---
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// --- WPILib Imports ---
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// --- Project-specific Imports ---
import frc.robot.Constants.DriveConstants;
import frc.robot.util.NavXPigeon2;

public class CANDriveSubsystem extends SubsystemBase {

    // --- Hardware ---
    private final TalonFX leftLeader = new TalonFX(DriveConstants.LEFT_LEADER_ID);
    private final TalonFX leftFollower = new TalonFX(DriveConstants.LEFT_FOLLOWER_ID);
    private final TalonFX rightLeader = new TalonFX(DriveConstants.RIGHT_LEADER_ID);
    private final TalonFX rightFollower = new TalonFX(DriveConstants.RIGHT_FOLLOWER_ID);
    private final NavXPigeon2 pigeon = new NavXPigeon2(); // Using the wrapper

    // --- Controls ---
    private final MotionMagicVelocityVoltage mmvLeft = new MotionMagicVelocityVoltage(0).withSlot(0);
    private final MotionMagicVelocityVoltage mmvRight = new MotionMagicVelocityVoltage(0).withSlot(0);

    // --- Kinematics ---
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH_METERS);
    
    // --- SIMULATION ---
    private DifferentialDrivetrainSim driveSim;
    // Simulation state variables
    private double simLeftRps = 0.0;
    private double simRightRps = 0.0;

    public CANDriveSubsystem() {
        // Configure motors
        configureDriveTalon(leftLeader, false); // Left side not inverted
        configureDriveTalon(rightLeader, true); // Right side inverted for tank drive

        // Set up followers
        leftFollower.setControl(new Follower(leftLeader.getDeviceID(), false));
        rightFollower.setControl(new Follower(rightLeader.getDeviceID(), false));

        // Reset sensors
        resetSensors();

        // --- SIMULATION SETUP ---
        if (RobotBase.isSimulation()) {
            driveSim = new DifferentialDrivetrainSim(
                DCMotor.getFalcon500(2),
                DriveConstants.DRIVE_GEARING,
                7.5, // MOI kg*m^2 (adjust as needed)
                60.0, // Mass kg (adjust as needed)
                DriveConstants.WHEEL_RADIUS_METERS,
                DriveConstants.TRACK_WIDTH_METERS,
                null // Standard deviations, can be null
            );
        }
    }

    private void configureDriveTalon(TalonFX talon, boolean invert) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        // The position and velocity signals will be scaled to wheel rotations
        cfg.Feedback.SensorToMechanismRatio = 1.0;

        // Configure velocity PID gains and feedforward
        var slot0 = new Slot0Configs();
        slot0.kS = DriveConstants.kS;
        slot0.kV = DriveConstants.kV;
        slot0.kA = DriveConstants.kA;
        slot0.kP = DriveConstants.kP_VELOCITY;
        slot0.kI = DriveConstants.kI_VELOCITY;
        slot0.kD = DriveConstants.kD_VELOCITY;
        cfg.Slot0 = slot0;

        // Configure Motion Magic acceleration (in RPS per second)
        var motionMagic = new MotionMagicConfigs();
        motionMagic.MotionMagicAcceleration = mpsToWheelRps(DriveConstants.MAX_ACCELERATION_MPS_SQ);
        cfg.MotionMagic = motionMagic;

        // Set motor inversion and neutral mode
        cfg.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        talon.getConfigurator().apply(cfg);
    }
    
    // --- SENSOR GETTERS ---

    public double getLeftDistanceMeters() {
        // getPosition() returns mechanism rotations (wheel rotations). Convert to meters.
        return leftLeader.getPosition().getValueAsDouble() * DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
    }

    public double getRightDistanceMeters() {
        // getPosition() returns mechanism rotations (wheel rotations). Convert to meters.
        return rightLeader.getPosition().getValueAsDouble() * DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
    }

    public Rotation2d getRotation2d() {
        // The wrapper handles converting NavX CW-positive to WPILib's CCW-positive standard.
        return pigeon.getRotation2d();
    }
    
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        // getVelocity() returns mechanism RPS (wheel RPS). Convert to m/s.
        double leftRps = leftLeader.getVelocity().getValueAsDouble();
        double rightRps = rightLeader.getVelocity().getValueAsDouble();
        return new DifferentialDriveWheelSpeeds(
            wheelRpsToMps(leftRps), 
            wheelRpsToMps(rightRps));
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getWheelSpeeds());
    }

    // --- CONTROL METHODS ---

    /**
     * Drives the robot using arcade-style controls with input squaring for finer control.
     * @param fwd The forward/backward speed (-1 to 1).
     * @param rot The rotation speed, where positive is counter-clockwise (-1 to 1).
     */
    public void arcadeDrive(double fwd, double rot) {
        // Square inputs while preserving sign for finer control at low speeds
        double fwdSpeed = Math.copySign(fwd * fwd, fwd);
        double rotSpeed = Math.copySign(rot * rot, rot);

        // Convert normalized inputs to physical units
        double vx = fwdSpeed * DriveConstants.MAX_SPEED_MPS;
        double wz = rotSpeed * DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC;
        
        setChassisSpeeds(new ChassisSpeeds(vx, 0.0, wz));
    }

    /**
     * Controls the robot using ChassisSpeeds. This is the primary method for autonomous path following.
     * @param speeds The desired robot speeds (forward, sideways, and angular).
     */
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        wheelSpeeds.desaturate(DriveConstants.MAX_SPEED_MPS);

        double leftRps = mpsToWheelRps(wheelSpeeds.leftMetersPerSecond);
        double rightRps = mpsToWheelRps(wheelSpeeds.rightMetersPerSecond);

        leftLeader.setControl(mmvLeft.withVelocity(leftRps));
        rightLeader.setControl(mmvRight.withVelocity(rightRps));

        if (RobotBase.isSimulation()) {
            simLeftRps = leftRps;
            simRightRps = rightRps;
        }
    }

    public void stop() {
        leftLeader.setControl(mmvLeft.withVelocity(0));
        rightLeader.setControl(mmvRight.withVelocity(0));
        if (RobotBase.isSimulation()) {
            simLeftRps = 0.0;
            simRightRps = 0.0;
        }
    }

    // --- UTILITY METHODS ---

    public void resetSensors() {
        leftLeader.setPosition(0);
        rightLeader.setPosition(0);
        pigeon.reset();
    }

    private static double mpsToWheelRps(double metersPerSecond) {
        return metersPerSecond / DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
    }
    
    private static double wheelRpsToMps(double wheelRps) {
        return wheelRps * DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Drive Distance (m)", getLeftDistanceMeters());
        SmartDashboard.putNumber("Right Drive Distance (m)", getRightDistanceMeters());
        SmartDashboard.putNumber("Robot Heading (deg)", getRotation2d().getDegrees());
        SmartDashboard.putNumber("Left Drive Velocity (RPS)", leftLeader.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Right Drive Velocity (RPS)", rightLeader.getVelocity().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        // --- Update Simulation Model ---
        driveSim.setInputs(
            leftLeader.getMotorVoltage().getValueAsDouble(),
            rightLeader.getMotorVoltage().getValueAsDouble()
        );
        driveSim.update(0.02);

        // --- Update Simulated Sensors ---
        // Update gyro. Sim heading is already CCW+ so no negation is needed.
        pigeon.setYaw(driveSim.getHeading());
        
        // Feed mechanism (wheel) sensor data back to simulated Talons
        double leftPosRots = driveSim.getLeftPositionMeters() / DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
        double rightPosRots = driveSim.getRightPositionMeters() / DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
        
        leftLeader.getSimState().setRawRotorPosition(leftPosRots);
        leftLeader.getSimState().setRotorVelocity(simLeftRps);
        rightLeader.getSimState().setRawRotorPosition(rightPosRots);
        rightLeader.getSimState().setRotorVelocity(simRightRps);

        RoboRioSim.setVInVoltage(RobotController.getBatteryVoltage());
    }
}

