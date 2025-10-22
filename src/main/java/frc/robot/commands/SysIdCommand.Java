package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import edu.wpi.first.wpilibj.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.sysid.SysIdRoutine.Direction;

/**
 * A command that provides the individual characterization tests for the SysId tool.
 * This class creates SysIdRoutine commands that can be triggered by name from the SysId application.
 *
 * @see <a href="https://docs.wpilib.org/en/stable/docs/software/pathplanning/sysid/characterizing-your-robot.html">SysId Docs</a>
 */
public class SysIdCommand {

    private final SysIdRoutine sysIdRoutine;

    /**
     * Creates a new factory for SysId commands.
     * @param drive The drivetrain subsystem to characterize.
     */
    public SysIdCommand(CANDriveSubsystem drive) {
        // The SysIdRoutine utility needs references to the methods that will execute the test.
        // We use lambdas here to pass our methods to the routine.
        sysIdRoutine = new SysIdRoutine(
            // Configuration for the routine (we can use defaults)
            new SysIdRoutine.Config(),
            // The "mechanism" to characterize, which holds the methods to call
            new SysIdRoutine.Mechanism(
                // This method is called by the routine to apply voltage to the motors
                (voltage) -> drive.setVoltage(voltage.getAsDouble(), voltage.getAsDouble()),
                // This method is called by the routine to log the state of the motors
                drive::logSysId,
                // This is the subsystem requirement
                drive
            )
        );
    }

    /**
     * Returns a command that will run the quasistatic (slow) test in the forward direction.
     */
    public Command quasistaticForward() {
        return sysIdRoutine.quasistatic(Direction.kForward);
    }

    /**
     * Returns a command that will run the quasistatic (slow) test in the reverse direction.
     */
    public Command quasistaticReverse() {
        return sysIdRoutine.quasistatic(Direction.kReverse);
    }

    /**
     * Returns a command that will run the dynamic (fast) test in the forward direction.
     */
    public Command dynamicForward() {
        return sysIdRoutine.dynamic(Direction.kForward);
    }

    /**
     * Returns a command that will run the dynamic (fast) test in the reverse direction.
     */
    public Command dynamicReverse() {
        return sysIdRoutine.dynamic(Direction.kReverse);
    }
}

