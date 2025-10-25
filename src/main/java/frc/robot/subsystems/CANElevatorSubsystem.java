package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;

import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class CANElevatorSubsystem extends SubsystemBase {

    private final SparkMax leftElevatorMotor;
    private final SparkMax rightElevatorMotor;

    private final SparkMaxConfig leftConfig;
    private final SparkMaxConfig rightConfig;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private final SparkClosedLoopController pidController;
    private double setpoint;

    public final DoubleSubscriber kP = DogLog.tunable("/Elevator/P", ElevatorConstants.kP);
    public final DoubleSubscriber kG = DogLog.tunable("/Elevator/G", ElevatorConstants.kG);

    public final DoubleSubscriber kMaxVelocityRPS = DogLog.tunable("/Elevator/Max Velocity RPS", 350.0);
    public final DoubleSubscriber kMaxAccelRPS2 = DogLog.tunable("/Elevator/Max Accel RPS2", 120.0);

    public ProfiledPIDController trapezoidController = new ProfiledPIDController(kP.get(), 0, 0, new Constraints(kMaxVelocityRPS.get(), kMaxAccelRPS2.get()));

    public void applyConfigs() {
        trapezoidController = new ProfiledPIDController(kP.get(), 0, 0, new Constraints(kMaxVelocityRPS.get(), kMaxAccelRPS2.get()));
        // Configure PID gains from constants
        // leftConfig.closedLoop
        // .apply(new MAXMotionConfig()
        // .maxVelocity(kMaxVelocityRPS.get())
        // .maxAcceleration(kMaxAccelRPS2.get()));
        // leftConfig.closedLoop.outputRange(ElevatorConstants.kMIN_OUTPUT, ElevatorConstants.kMAX_OUTPUT);
        leftConfig.closedLoop.pidf(kP.get(), ElevatorConstants.kI, ElevatorConstants.kD, kG.get());
        leftConfig.idleMode(IdleMode.kBrake);
        rightConfig.idleMode(IdleMode.kBrake);
        rightConfig.follow(leftElevatorMotor, true);

        leftElevatorMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightElevatorMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public CANElevatorSubsystem() {
        leftElevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_LEADER_ID, MotorType.kBrushless);
        rightElevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_FOLLOWER_ID, MotorType.kBrushless);

        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();

        applyConfigs();

        leftEncoder = leftElevatorMotor.getEncoder();
        rightEncoder = rightElevatorMotor.getEncoder();

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        pidController = leftElevatorMotor.getClosedLoopController();
    }

    @Override
    public void periodic()
    {
        double position = leftEncoder.getPosition();
        DogLog.log("Elevator Position", position);
        double speed = trapezoidController.calculate(position, setpoint);
        leftElevatorMotor.set(speed);
    }

    public void increaseSetpoint() {
        setpoint++;
    }

    public void decreaseSetpoint() {
        setpoint--;
    }

    /**
     * Commands the elevator to move to a specific position in rotations.
     * @param targetRotations The target position for the elevator in rotations.
     */
    public void setPosition(double targetRotations) {
        this.setpoint = targetRotations;
        // Use kG for gravity feedforward

        // pidController.setReference(targetRotations, ControlType.kPosition);
    }

    /**
     * @return The current position of the elevator encoder in rotations.
     */
    public double getPosition() {
        return leftEncoder.getPosition();
    }

    /**
     * Checks if the elevator is at its target setpoint within a defined tolerance.
     * @return True if the elevator is at the setpoint, false otherwise.
     */
    public boolean atSetpoint() {
        // You may need to adjust the tolerance value in Constants.java
        return Math.abs(leftEncoder.getPosition() - this.setpoint) < ElevatorConstants.kPOSITION_TOLERANCE;
    }

    public void raise()
    {
        leftElevatorMotor.set(ElevatorConstants.MANUAL_RAISE_SPEED);
    }

    public void lower()
    {
        leftElevatorMotor.set(ElevatorConstants.MANUAL_LOWER_SPEED);
    }

    public void stop()
    {
        leftElevatorMotor.setVoltage(0);
    }
}
