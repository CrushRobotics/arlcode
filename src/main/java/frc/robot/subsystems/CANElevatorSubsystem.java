package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

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

    public CANElevatorSubsystem() {
        leftElevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_LEADER_ID, MotorType.kBrushless);
        rightElevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_FOLLOWER_ID, MotorType.kBrushless);

        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();
        
        leftConfig.idleMode(IdleMode.kBrake);
        // Configure PID gains from constants
        leftConfig.closedLoop.pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
        leftConfig.closedLoop.outputRange(ElevatorConstants.kMIN_OUTPUT, ElevatorConstants.kMAX_OUTPUT);

        rightConfig.idleMode(IdleMode.kBrake);
        rightConfig.follow(leftElevatorMotor, true);

        leftElevatorMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rightElevatorMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        leftEncoder = leftElevatorMotor.getEncoder();
        rightEncoder = rightElevatorMotor.getEncoder();
        
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        pidController = leftElevatorMotor.getClosedLoopController();
    }

    @Override 
    public void periodic()
    {
        SmartDashboard.putNumber("Elevator Position", leftEncoder.getPosition());
    }

    /**
     * Commands the elevator to move to a specific position in rotations.
     * @param targetRotations The target position for the elevator in rotations.
     */
    public void setPosition(double targetRotations) {
        this.setpoint = targetRotations;
        pidController.setReference(targetRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0, ElevatorConstants.kF);
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
        leftElevatorMotor.set(0.4); 
    }

    public void lower() 
    {
        leftElevatorMotor.set(-0.3);
    }

    public void stop() 
    {
        leftElevatorMotor.setVoltage(0);
    }
}

