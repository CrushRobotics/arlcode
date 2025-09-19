package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;

public class CANAlgaeIntakeSubsystem extends SubsystemBase {
    private final SparkMax algaeIntakeMotor;
    private final SparkMaxConfig config;
    private final RelativeEncoder encoder;

    public CANAlgaeIntakeSubsystem() {
        // Using a constant for the CAN ID is best practice.
        algaeIntakeMotor = new SparkMax(AlgaeIntakeConstants.ALGAE_INTAKE_ID, MotorType.kBrushed);
        
        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        
        algaeIntakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder = algaeIntakeMotor.getEncoder();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Algae Intake Position", encoder.getPosition());
        SmartDashboard.putNumber("Algae Intake Velocity", encoder.getVelocity());
    }

    public void right() {
        algaeIntakeMotor.set(0.6);
    }

    public void left() {
        algaeIntakeMotor.set(-0.6);
    }

    public void stop() {
        algaeIntakeMotor.set(0.0);
    }

    public void moveToShootPosition() {
        // TODO: Implement shooting position logic
        // setTarget(Constants.ArmConstants.SHOOTING_POSITION);
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }
}

