package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.CANDriveSubsystem;

/**
 * A command that runs the SysId routine for the drivetrain.
 * This command will automatically run the quasistatic and dynamic tests
 * in both forward and reverse directions.
 */
public class RunSysIDTests extends SequentialCommandGroup {
  
  public RunSysIDTests(CANDriveSubsystem drive) {

    // The SysIdRoutine constructor requires a config object and a mechanism object.
    // The config object specifies how the tests will be run (e.g., voltage ramp rate).
    // The mechanism object provides the routine with methods to control and measure the drivetrain.
    SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            // This is the consumer for the applied voltage. We're giving SysId direct access
            // to our new setVoltage method in the drivetrain subsystem.
            (voltage) -> drive.setVoltage(voltage.in(Units.Volts), voltage.in(Units.Volts)),
            
            // This is the logger for the mechanism state. SysId needs to know the position
            // and velocity of both sides of the drivetrain to analyze the data.
            log -> {
              log.motor("drive-left")
                  .voltage(drive.getLeftLeader().getMotorVoltage().getValue())
                  .angularPosition(drive.getLeftLeader().getPosition().getValue())
                  .angularVelocity(drive.getLeftLeader().getVelocity().getValue());
              log.motor("drive-right")
                  .voltage(drive.getRightLeader().getMotorVoltage().getValue())
                  .angularPosition(drive.getRightLeader().getPosition().getValue())
                  .angularVelocity(drive.getRightLeader().getVelocity().getValue());
            },
            drive
        )
    );

    // Add the individual test commands to the sequence.
    // This will run the full characterization routine automatically.
    addCommands(
        sysIdRoutine.quasistatic(Direction.kForward),
        sysIdRoutine.quasistatic(Direction.kReverse),
        sysIdRoutine.dynamic(Direction.kForward),
        sysIdRoutine.dynamic(Direction.kReverse)
    );
  }
}
