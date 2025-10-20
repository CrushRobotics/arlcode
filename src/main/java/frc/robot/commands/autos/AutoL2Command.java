package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.SetScoringPositionCommand;
import frc.robot.commands.CoralIntakeCommand.CoralIntakeDirection;
import frc.robot.commands.SetScoringPositionCommand.ScoringLevel;
import frc.robot.subsystems.CANArmSubsystem;
import frc.robot.subsystems.CANCoralIntakeSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANElevatorSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.ReefState;
import frc.robot.subsystems.VisionSubsystem;

/**
 * An autonomous command that uses the AutoAlignCommand to find the best target,
 * align to it, and then score a pre-loaded piece on the L2 peg.
 */
public class AutoL2Command extends SequentialCommandGroup {

    public AutoL2Command(
        CANDriveSubsystem driveSubsystem,
        LocalizationSubsystem localizationSubsystem,
        VisionSubsystem visionSubsystem,
        ReefState reefState,
        CANArmSubsystem armSubsystem,
        CANElevatorSubsystem elevatorSubsystem,
        CANCoralIntakeSubsystem coralIntakeSubsystem
    ) {
        addCommands(
            // 1. Dynamically find the best scoring location and align the robot to it.
            new AutoAlignCommand(driveSubsystem, localizationSubsystem, visionSubsystem, reefState),

            // 2. Once aligned, move the arm and elevator to the L2 scoring position.
            new SetScoringPositionCommand(armSubsystem, elevatorSubsystem, ScoringLevel.L2),

            // 3. Run the coral intake downwards to score the piece.
            //    withTimeout(1.5) ensures it runs for a fixed duration.
            new CoralIntakeCommand(coralIntakeSubsystem, CoralIntakeDirection.Down).withTimeout(1.5)
        );
    }
}
