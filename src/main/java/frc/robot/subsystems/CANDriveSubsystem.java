package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LocalizationConstants;
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

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(LocalizationConstants.TRACK_WIDTH_METERS);

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
        return rightLeader.getPosition().getValueAsDouble() * DriveConstants.ROTATIONS_TO_METERS;
    }

    public Rotation2d getRotation2d() {
        // THE FIX: Negate the NavX angle to conform to WPILib's CCW-positive convention.
        return Rotation2d.fromDegrees(-navX.getAngle());
    }

    public double getHeading() {
        return getRotation2d().getDegrees();
    }
    
    public double getForwardVelocityMetersPerSec() {
        double leftRPS = leftLeader.getVelocity().getValueAsDouble();
        double rightRPS = rightLeader.getVelocity().getValueAsDouble();
        return ((leftRPS + rightRPS) / 2.0) * DriveConstants.ROTATIONS_TO_METERS;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            (leftLeader.getVelocity().getValueAsDouble() * DriveConstants.ROTATIONS_TO_METERS),
            (rightLeader.getVelocity().getValueAsDouble() * DriveConstants.ROTATIONS_TO_METERS)
        );
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getWheelSpeeds());
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        drive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond, true);
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Drive Distance (m)", getLeftDistanceMeters());
        SmartDashboard.putNumber("Right Drive Distance (m)", getRightDistanceMeters());
        SmartDashboard.putNumber("Robot Heading (deg)", getHeading());
    }

    /**
     * Drives the robot using arcade-style controls.
     * @param fwd The forward/backward speed (-1 to 1).
     * @param rot The rotation speed, where positive is counter-clockwise (-1 to 1).
     */
    public void drive(double fwd, double rot) {
        // THE FIX: Correct the arcade drive logic for a CCW-positive system.
        // A positive 'rot' should result in a counter-clockwise turn.
        drive(fwd - rot, fwd + rot, false);
    }

    private void drive(double left, double right, boolean isVelocity) {
        double leftOutput = left;
        double rightOutput = right;

        if (!isVelocity) {
            // Scale down joystick input for better control
            leftOutput *= 0.65;
            rightOutput *= 0.65;
        }

        leftOut.Output = leftOutput;
        rightOut.Output = rightOutput;

        leftLeader.setControl(leftOut);
        rightLeader.setControl(rightOut);
    }
}
