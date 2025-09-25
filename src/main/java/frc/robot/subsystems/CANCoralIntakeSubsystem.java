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
import frc.robot.Constants.CoralIntakeConstants;

public class CANCoralIntakeSubsystem extends SubsystemBase {
    public final SparkMax coralIntakeMotor;
    private final SparkMaxConfig config;
    private final RelativeEncoder encoder;

    public CANCoralIntakeSubsystem() {
        coralIntakeMotor = new SparkMax(CoralIntakeConstants.CORAL_INTAKE_ID, MotorType.kBrushless);
        
        config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake);
        coralIntakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = coralIntakeMotor.getEncoder();
    }

    @Override
    public void periodic() {
        // Publishes encoder data to the SmartDashboard, resolving the "unused field" warning.
        SmartDashboard.putNumber("Coral Intake Position", encoder.getPosition());
    }

    public void right() {
        coralIntakeMotor.set(CoralIntakeConstants.CORAL_INTAKE_SPEED);
    }
    public void moveToShootPosition(){
        // TODO: Implement shooting position logic
        //setTarget(frc.robot.Constants.ArmConstants.SHOOTING_POSITION);
    }

    public void left() {
        coralIntakeMotor.set(-CoralIntakeConstants.CORAL_INTAKE_SPEED);
    }
    public void stop(){
        coralIntakeMotor.setVoltage(0);
    }
}
