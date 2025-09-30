package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkBase.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    
    private SparkMax leftClimber;
    private SparkMax rightClimber;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;
    

    public ClimberSubsystem() {
        leftClimber = new SparkMax(27, MotorType.kBrushless);
        rightClimber = new SparkMax(24, MotorType.kBrushless);

        // leftClimber.restoreFactoryDefaults();
        // rightClimber.restoreFactoryDefaults();

        leftClimber.setInverted(true);
        
        //leftClimber.setIdleMode(IdleMode.kBrake);
        // rightClimber.setIdleMode(IdleMode.kBrake);

        leftEncoder = leftClimber.getEncoder();
        rightEncoder = rightClimber.getEncoder();
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("left climber encoder", leftEncoder.getPosition());
        SmartDashboard.putNumber("right climber encoder", rightEncoder.getPosition());
    }

    public void Climb(){
        if(isInUpperBound()) {
            leftClimber.set(0.45);
            rightClimber.set(0.45);
        }
        else {
            leftClimber.set(0);
            rightClimber.set(0);
        }
    }

    public void Lower() {
        if(isInLowerBound()) {    
            leftClimber.set(-1);
            rightClimber.set(-1);
        }
        else {
            leftClimber.set(0);
            rightClimber.set(0);
        }
    }

    public void stop() {
        leftClimber.set(0);
        rightClimber.set(0);
    }

    public boolean isInUpperBound() {
        
        return leftEncoder.getPosition() < frc.robot.Constants.ClimberConstants.MAX_BOUND;
    }
    public boolean isInLowerBound() {
        return leftEncoder.getPosition() > frc.robot.Constants.ClimberConstants.MIN_BOUND;
    }
}
