package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.TankDriveSubsystem;

public class TeleopDriveCommand extends Command {

    private final TankDriveSubsystem driveSubsystem;
    private final DoubleSupplier fwdSupplier;
    private final DoubleSupplier rotSupplier;

    /**
     * Creates a new TeleopDriveCommand. This command will take over the drivetrain
     * by default and apply arcade-style controls from the provided suppliers.
     *
     * @param driveSubsystem The drive subsystem to control.
     * @param fwdSupplier    A DoubleSupplier that provides the forward/backward input (-1 to 1).
     * @param rotSupplier    A DoubleSupplier that provides the rotation input (-1 to 1).
     */
    public TeleopDriveCommand(TankDriveSubsystem driveSubsystem, DoubleSupplier fwdSupplier, DoubleSupplier rotSupplier) {
        this.driveSubsystem = driveSubsystem;
        this.fwdSupplier = fwdSupplier;
        this.rotSupplier = rotSupplier;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        // 1. Get the raw input from the suppliers
        double fwd = fwdSupplier.getAsDouble();
        double rot = rotSupplier.getAsDouble();

        // 2. Apply a deadzone to the inputs
        fwd = Math.abs(fwd) > OperatorConstants.CONTROLLER_DEADZONE ? fwd : 0.0;
        rot = Math.abs(rot) > OperatorConstants.CONTROLLER_DEADZONE ? rot : 0.0;

        // 3. Apply cubic scaling for finer control at low speeds
        fwd = Math.pow(fwd, 3);
        rot = Math.pow(rot, 3);
        
        // 4. Pass the processed values to the drive subsystem
        driveSubsystem.arcadeDriveNormalized(fwd, rot);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends
        driveSubsystem.stop();
    }
}
