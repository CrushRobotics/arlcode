package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
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

    public CANElevatorSubsystem() {
        leftElevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_LEADER_ID, MotorType.kBrushless);
        rightElevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_FOLLOWER_ID, MotorType.kBrushless);

        // Instead of calling restoreFactoryDefaults directly, we create fresh configs
        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();
        
        // Set idle mode on the configuration object
        leftConfig.idleMode(IdleMode.kBrake);
        rightConfig.idleMode(IdleMode.kBrake);

        // Set the follower on the configuration object
        rightConfig.follow(leftElevatorMotor, true);

        // Apply the configurations to the motor controllers
        leftElevatorMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rightElevatorMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        leftEncoder = leftElevatorMotor.getEncoder();
        rightEncoder = rightElevatorMotor.getEncoder();
        
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    @Override 
    public void periodic()
    {
        SmartDashboard.putNumber("Elevator Position", leftEncoder.getPosition());
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

