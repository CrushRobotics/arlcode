package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.CANArmSubsystem;
import frc.robot.subsystems.CANElevatorSubsystem;

/**
 * A command to move the arm and elevator to a preset scoring position (L2 or L3).
 */
public class SetScoringPositionCommand extends Command {

  /**
   * Defines the scoring levels for the preset positions.
   */
  public enum ScoringLevel {
    L2,
    L3
  }

  private final CANArmSubsystem armSubsystem;
  private final CANElevatorSubsystem elevatorSubsystem;
  private final ScoringLevel level;

  private double armTarget;
  private double elevatorTarget;

  /**
   * Creates a new SetScoringPositionCommand.
   *
   * @param armSubsystem      The arm subsystem to control.
   * @param elevatorSubsystem The elevator subsystem to control.
   * @param level             The target scoring level (L2 or L3).
   */
  public SetScoringPositionCommand(CANArmSubsystem armSubsystem, CANElevatorSubsystem elevatorSubsystem, ScoringLevel level) {
    this.armSubsystem = armSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.level = level;
    addRequirements(armSubsystem, elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the target positions based on the selected scoring level from Constants
    switch (level) {
      case L2:
        armTarget = ArmConstants.L2_POSITION_ROTATIONS;
        elevatorTarget = ElevatorConstants.L2_POSITION_ROTATIONS;
        break;
      case L3:
        armTarget = ArmConstants.L3_POSITION_ROTATIONS;
        elevatorTarget = ElevatorConstants.L3_POSITION_ROTATIONS;
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Command the subsystems to move to their target positions
    armSubsystem.setPosition(armTarget);
    elevatorSubsystem.setPosition(elevatorTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // It's good practice to stop the motors, although the PID controller
    // should hold them at the setpoint.
    armSubsystem.stop();
    elevatorSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // The command is finished when both the arm and elevator are at their setpoints.
    return armSubsystem.atSetpoint() && elevatorSubsystem.atSetpoint();
  }
}
