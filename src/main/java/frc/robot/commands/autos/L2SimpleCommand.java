package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.SetScoringPositionCommand;
import frc.robot.commands.CoralIntakeCommand.CoralIntakeDirection;
import frc.robot.commands.SetScoringPositionCommand.ScoringLevel;
import frc.robot.subsystems.CANArmSubsystem;
import frc.robot.subsystems.CANCoralIntakeSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANElevatorSubsystem;

/**
 * A simple, time-based autonomous command that drives forward, moves to the L2
 * scoring position, and outtakes a coral piece.
 */
public class L2SimpleCommand extends SequentialCommandGroup {
  public L2SimpleCommand(
      CANDriveSubsystem drive,
      CANArmSubsystem arm,
      CANElevatorSubsystem elevator,
      CANCoralIntakeSubsystem coralIntake) {

    addCommands(
        // 1. Drive forward for a set duration
        new RunCommand(() -> drive.arcadeDrive(AutoConstants.AUTO_DRIVE_SPEED, 0), drive).withTimeout(AutoConstants.AUTO_DRIVE_SECONDS),

        // 2. Move arm and elevator to L2 position
        new SetScoringPositionCommand(arm, elevator, ScoringLevel.L2),

        // 3. Outtake the coral for 1 second
        new CoralIntakeCommand(coralIntake, CoralIntakeDirection.Down).withTimeout(1.0)
    );
  }
}
