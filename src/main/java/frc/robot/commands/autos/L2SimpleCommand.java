package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.SetScoringPositionCommand;
import frc.robot.commands.CoralIntakeCommand.CoralIntakeDirection;
import frc.robot.commands.SetScoringPositionCommand.ScoringLevel;
import frc.robot.subsystems.CANArmSubsystem;
import frc.robot.subsystems.CANCoralIntakeSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANElevatorSubsystem;

/**
 * A simple autonomous command that drives forward for a set time,
 * moves the arm and elevator to the L2 scoring position, and then outtakes the coral.
 */
public class L2SimpleCommand extends SequentialCommandGroup {

    public L2SimpleCommand(
        CANDriveSubsystem driveSubsystem,
        CANArmSubsystem armSubsystem,
        CANElevatorSubsystem elevatorSubsystem,
        CANCoralIntakeSubsystem coralIntakeSubsystem
    ) {
        addCommands(
            // 1. Drive forward for a set amount of time using the existing AutoCommand
            new AutoCommand(driveSubsystem),

            // 2. Move the arm and elevator to the L2 scoring position
            new SetScoringPositionCommand(armSubsystem, elevatorSubsystem, ScoringLevel.L2),

            // 3. Run the coral intake downwards to score the piece.
            //    withTimeout(1.5) will make it run for 1.5 seconds and then stop.
            new CoralIntakeCommand(coralIntakeSubsystem, CoralIntakeDirection.Down).withTimeout(1.5)
        );
    }
}

