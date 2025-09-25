package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANAlgaeSubsystem;
import frc.robot.commands.AlgaeIntakeCommand.AlgaeIntakeDirection;
import frc.robot.subsystems.CANAlgaeIntakeSubsystem;

public class AlgaeCommand extends Command {
    public static enum AlgaeDirection  {
        Up,
        Down
    }

    private final CANAlgaeSubsystem algaeSubsystem;
    private final CANAlgaeIntakeSubsystem algaeIntakeSubsystem;
    private AlgaeDirection direction;
    private AlgaeIntakeDirection algaeIntakeDirection;

    public AlgaeCommand (CANAlgaeSubsystem AlgaeSubsystem, AlgaeDirection direction, CANAlgaeIntakeSubsystem algaeIntakeSubsystem)
    {
        this.direction = direction;
        this.algaeSubsystem = AlgaeSubsystem;
        addRequirements(AlgaeSubsystem);
        this.algaeIntakeSubsystem = algaeIntakeSubsystem;
        addRequirements(algaeIntakeSubsystem);
    }
    

    @Override
    public void execute()
    {
        if (direction == AlgaeDirection.Up){
          algaeSubsystem.setMotorPostion(-3.3); // algaesubsystem.setmotorpostion
          algaeIntakeSubsystem.stop();
        } else {
          algaeSubsystem.setMotorPostion(-7.4); // put in algae intake subsystem command
          algaeIntakeSubsystem.right();
        }
    }

    @Override
    public void end (boolean isInterrupted)
    {
      algaeSubsystem.stop();
      // **FIXED**: Ensure the intake motor is stopped when the command ends.
      algaeIntakeSubsystem.stop();
    }
}
