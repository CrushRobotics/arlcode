package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot; // Import the ClosedLoopSlot enum from its new location
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;

import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class CANArmSubsystem extends SubsystemBase {

    private final SparkMax armMotor;
    private final SparkMaxConfig config;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;
    private double setpoint;

    public final DoubleSubscriber kP = DogLog.tunable("/Arm/P", ArmConstants.kP);
    public final DoubleSubscriber kMaxVelocityRPS = DogLog.tunable("/Arm/Max Velocity RPS", 400.0);
    public final DoubleSubscriber kMaxAccelRPS2 = DogLog.tunable("/Arm/Max Accel RPS2", 120.0);

    public ProfiledPIDController trapezoidController = new ProfiledPIDController(kP.get(), 0, 0, new Constraints(kMaxVelocityRPS.get(), kMaxAccelRPS2.get()));

    public void applyConfigs() {
        trapezoidController = new ProfiledPIDController(kP.get(), 0, 0, new Constraints(kMaxVelocityRPS.get(), kMaxAccelRPS2.get()));
        // Configure PID gains and add the Feedforward term from constants
        // This is the new way to configure PID constants for 2025
        // config.closedLoop
        // .apply(new MAXMotionConfig()
        // .maxVelocity(kMaxVelocityRPS.get())
        // .maxAcceleration(kMaxAccelRPS2.get()));

        config.closedLoop.pid(kP.get(), ArmConstants.kI, ArmConstants.kD);
        config.idleMode(IdleMode.kBrake);
        // The .ff() method is removed; feedforward is now applied in setReference()
        // We will use kG as the feedforward term for position control.
        // config.closedLoop.outputRange(ArmConstants.kMIN_OUTPUT, ArmConstants.kMAX_OUTPUT);

        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public CANArmSubsystem() {
        armMotor = new SparkMax(ArmConstants.CORAL_ARM_ID, MotorType.kBrushless);

        config = new SparkMaxConfig();
        applyConfigs();

        // The method to get the controller has also been renamed
        pidController = armMotor.getClosedLoopController();
        encoder = armMotor.getEncoder();
    }

    @Override
    public void periodic() {
        double position = encoder.getPosition();
        DogLog.log("Arm Position", position);
        DogLog.log("Arm Setpoint", setpoint);
        double speed = trapezoidController.calculate(position, setpoint);
        armMotor.set(speed);
    }

    /**
     * Commands the arm to move to a specific position in rotations.
     * The PID controller on the SparkMax will now hold this position.
     * @param targetRotations The target position for the arm in rotations.
     */
    public void setPosition(double targetRotations) {
        this.setpoint = targetRotations;
        // The arbitrary feedforward term (kG) is now passed into setReference.
        // The '0' is replaced with the correct enum type 'ClosedLoopSlot.kSlot0'.
        // pidController.setReference(targetRotations, ControlType.kPosition);
    }

    public void increaseSetpoint() {
        setpoint++;
    }

    public void decreaseSetpoint() {
        setpoint--;
    }
    /**
     * A method to hold the arm at its current position. It reads the current
     * encoder value and sets that as the new target setpoint.
     */
    // public void holdPosition() {
    //     setpoint = encoder.getPosition();
    //     pidController.setReference(setpoint, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, ArmConstants.kG);
    // }

    /**
     * @return The current position of the arm encoder in rotations.
     */
    public double getPosition() {
        return encoder.getPosition();
    }

    /**
     * Checks if the arm is at its target setpoint within a defined tolerance.
     * @return True if the arm is at the setpoint, false otherwise.
     */
    public boolean atSetpoint() {
        // You may need to adjust the tolerance value in Constants.java
        return Math.abs(encoder.getPosition() - this.setpoint) < ArmConstants.kPOSITION_TOLERANCE;
    }

    public void right() {
        armMotor.set(ArmConstants.MANUAL_ARM_SPEED);
    }

    public void left() {
        armMotor.set(-ArmConstants.MANUAL_ARM_SPEED);
    }

    public void stop(){
        armMotor.setVoltage(0);
    }
}
