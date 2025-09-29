package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot; // Import the ClosedLoopSlot enum from its new location
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class CANArmSubsystem extends SubsystemBase {
    private final SparkMax armMotor;
    private final SparkMaxConfig config;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;
    private double setpoint;

    public CANArmSubsystem() {
        armMotor = new SparkMax(ArmConstants.CORAL_ARM_ID, MotorType.kBrushless);
        
        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        
        // Configure PID gains and add the Feedforward term from constants
        // This is the new way to configure PID constants for 2025
        config.closedLoop.pid(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
        // The .ff() method is removed; feedforward is now applied in setReference()
        config.closedLoop.outputRange(ArmConstants.kMIN_OUTPUT, ArmConstants.kMAX_OUTPUT);

        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // The method to get the controller has also been renamed
        pidController = armMotor.getClosedLoopController();
        encoder = armMotor.getEncoder();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", encoder.getPosition());
        SmartDashboard.putNumber("Arm Setpoint", setpoint);
    }

    /**
     * Commands the arm to move to a specific position in rotations.
     * The PID controller on the SparkMax will now hold this position.
     * @param targetRotations The target position for the arm in rotations.
     */
    public void setPosition(double targetRotations) {
        this.setpoint = targetRotations;
        // The arbitrary feedforward term (kF) is now passed into setReference.
        // The '0' is replaced with the correct enum type 'ClosedLoopSlot.kSlot0'.
        pidController.setReference(targetRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0, ArmConstants.kF);
    }

    /**
     * A method to hold the arm at its current position. It reads the current
     * encoder value and sets that as the new target setpoint.
     */
    public void holdPosition() {
        setpoint = encoder.getPosition();
        pidController.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, ArmConstants.kF);
    }

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

