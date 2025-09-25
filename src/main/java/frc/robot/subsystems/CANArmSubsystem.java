package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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

        // Configure PID gains from constants
        config.closedLoop.pid(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
        config.closedLoop.outputRange(ArmConstants.kMIN_OUTPUT, ArmConstants.kMAX_OUTPUT);

        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = armMotor.getEncoder();
        pidController = armMotor.getClosedLoopController();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", encoder.getPosition());
    }

    /**
     * Commands the arm to move to a specific position in rotations.
     * @param targetRotations The target position for the arm in rotations.
     */
    public void setPosition(double targetRotations) {
        this.setpoint = targetRotations;
        pidController.setReference(targetRotations, ControlType.kPosition);
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
        armMotor.set(0.3);
    }

    public void left() {
        armMotor.set(-0.3);
    }

    public void stop(){
        armMotor.setVoltage(0);
    }
}

