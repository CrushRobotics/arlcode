package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;

public class CANDriveSubsystem extends SubsystemBase {

    private final TalonFX leftLeader = new TalonFX(DriveConstants.LEFT_LEADER_ID);
    private final TalonFX leftFollower = new TalonFX(DriveConstants.LEFT_FOLLOWER_ID);
    private final TalonFX rightLeader = new TalonFX(DriveConstants.RIGHT_LEADER_ID);
    private final TalonFX rightFollower = new TalonFX(DriveConstants.RIGHT_FOLLOWER_ID);

    private final DutyCycleOut leftOut = new DutyCycleOut(0);
    private final DutyCycleOut rightOut = new DutyCycleOut(0);

    private final AHRS navX = new AHRS(SPI.Port.kMXP);

    public CANDriveSubsystem() {
        var leftConfiguration = new TalonFXConfiguration();
        leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftLeader.getConfigurator().apply(leftConfiguration);
        leftFollower.getConfigurator().apply(leftConfiguration);

        var rightConfiguration = new TalonFXConfiguration();
        rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightLeader.getConfigurator().apply(rightConfiguration);
        rightFollower.getConfigurator().apply(rightConfiguration);

        leftFollower.setControl(new Follower(leftLeader.getDeviceID(), false));
        rightFollower.setControl(new Follower(rightLeader.getDeviceID(), false));

        // Reset encoders and gyro on startup
        leftLeader.setPosition(0);
        rightLeader.setPosition(0);
        navX.reset();
    }
    
    public double getLeftDistanceMeters() {
        return leftLeader.getPosition().getValueAsDouble() * DriveConstants.ROTATIONS_TO_METERS;
    }

    public double getRightDistanceMeters() {
        // Right side is often negative depending on motor mounting
        return rightLeader.getPosition().getValueAsDouble() * DriveConstants.ROTATIONS_TO_METERS;
    }

    public Rotation2d getRotation2d() {
        // THE FIX: The NavX getAngle() method is counter-clockwise positive,
        // which is the standard convention for WPILib. Removing the negation
        // ensures the gyro reading is consistent with the rest of the code.
        return Rotation2d.fromDegrees(navX.getAngle());
    }

    public double getHeading() {
        return getRotation2d().getDegrees();
    }
    
    /** Gets the current speed of the robot's wheels. Useful for the cost function. */
    public double getForwardVelocityMetersPerSec() {
        double leftRPS = leftLeader.getVelocity().getValueAsDouble();
        double rightRPS = rightLeader.getVelocity().getValueAsDouble();
        // Average the two sides and convert from rotations/sec to meters/sec
        return ((leftRPS + rightRPS) / 2.0) * DriveConstants.ROTATIONS_TO_METERS;
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Drive Distance (m)", getLeftDistanceMeters());
        SmartDashboard.putNumber("Right Drive Distance (m)", getRightDistanceMeters());
        SmartDashboard.putNumber("Robot Heading (deg)", getHeading());
    }

    public void drive(double fwd, double rot) {
        // Positive 'rot' value results in a counter-clockwise turn.
        leftOut.Output = (fwd + rot) * 0.65;
        rightOut.Output = (fwd - rot) * 0.65;

        leftLeader.setControl(leftOut);
        rightLeader.setControl(rightOut);
    }
}

