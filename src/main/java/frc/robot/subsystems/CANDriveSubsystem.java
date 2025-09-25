package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANDriveSubsystem extends SubsystemBase {
    private final CANBus kCANBus = new CANBus();

    private final TalonFX leftLeader = new TalonFX(8, kCANBus);
    private final TalonFX leftFollower = new TalonFX(9, kCANBus);
    private final TalonFX rightLeader = new TalonFX(7, kCANBus);
    private final TalonFX rightFollower = new TalonFX(6, kCANBus);

    public final DutyCycleOut leftOut = new DutyCycleOut(0);
    public final DutyCycleOut rightOut = new DutyCycleOut(0);

    public final XboxController joystick = new XboxController(0);

    public CANDriveSubsystem() {
        var leftConfiguration = new TalonFXConfiguration();
        var rightConfiguration = new TalonFXConfiguration();

        leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        leftLeader.getConfigurator().apply(leftConfiguration);
        leftFollower.getConfigurator().apply(leftConfiguration);
        rightLeader.getConfigurator().apply(rightConfiguration);
        rightFollower.getConfigurator().apply(rightConfiguration);

        leftFollower.setControl(new Follower(leftLeader.getDeviceID(), false));
        rightFollower.setControl(new Follower(rightLeader.getDeviceID(), false));

        leftLeader.setSafetyEnabled(true);
        rightLeader.setSafetyEnabled(true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
        // Get the velocity of the leader motors in Rotations Per Second.
        // The sign of the velocity indicates the direction (positive for forward, negative for reverse).
        double leftVelocityRPS = leftLeader.getVelocity().getValueAsDouble();
        double rightVelocityRPS = rightLeader.getVelocity().getValueAsDouble();

        // Send velocity data to the SmartDashboard
        SmartDashboard.putNumber("Left Drive Velocity (RPS)", leftVelocityRPS);
        SmartDashboard.putNumber("Right Drive Velocity (RPS)", rightVelocityRPS);

        // Determine a general direction string based on the average velocity
        double averageVelocity = (leftVelocityRPS + rightVelocityRPS) / 2.0;
        String direction;
        if (averageVelocity > 0.1) {
            direction = "Forward";
        } else if (averageVelocity < -0.1) {
            direction = "Reverse";
        } else {
            direction = "Stopped";
        }
        
        // Send the direction string to the SmartDashboard
        SmartDashboard.putString("Drivetrain Direction", direction);

        // Also send the commanded output percentages to the dashboard for debugging
        SmartDashboard.putNumber("Left Drive Output (%)", leftOut.Output);
        SmartDashboard.putNumber("Right Drive Output (%)", rightOut.Output);
    }


    public void drive(double fwd, double rot) {
        leftOut.Output = (fwd + rot) * 0.65;
        rightOut.Output = (fwd - rot) * 0.65;

        leftLeader.setControl(leftOut);
        rightLeader.setControl(rightOut);
    }
}

