package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CANArmSubsystem;
import frc.robot.subsystems.CANElevatorSubsystem;

public class UpdateMotorConfigs extends InstantCommand {
    public UpdateMotorConfigs(CANElevatorSubsystem elevator, CANArmSubsystem arm) {
        super(()-> {
            elevator.applyConfigs();
            arm.applyConfigs();
        });
    }
}
