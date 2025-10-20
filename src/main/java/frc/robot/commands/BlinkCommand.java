package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

public class BlinkCommand extends Command {

    private boolean isBlinking;

    public BlinkCommand()
    {
        isBlinking = false;
    }

    @Override
    public void execute() {
        super.execute();

        
        if (isBlinking)
        {
            // Set mode for BOTH cameras
            LimelightHelpers.setLEDMode_ForceOff("limelight-right");
            LimelightHelpers.setLEDMode_ForceOff("limelight-left");
            isBlinking = false;
        }
        else 
        {
            // Set mode for BOTH cameras
            LimelightHelpers.setLEDMode_ForceBlink("limelight-right");
            LimelightHelpers.setLEDMode_ForceBlink("limelight-left");
            isBlinking = true;
        }
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    
    
}
