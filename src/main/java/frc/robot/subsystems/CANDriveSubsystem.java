package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.XboxController;
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

    public void drive(double fwd, double rot) {
        leftOut.Output = (fwd + rot) * 0.65;
        rightOut.Output = (fwd - rot) * 0.65;

        leftLeader.setControl(leftOut);
        rightLeader.setControl(rightOut);
    }
}