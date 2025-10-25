package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.AutoAlignToCS;
import frc.robot.commands.AutoAlignToPipe;
import frc.robot.commands.AutoAlignCommand.AlignMode;
import frc.robot.commands.AutoAlignToCS.CoralStationSide;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.DriveToPose;
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
            Commands.runOnce(()-> {
                    if (RobotBase.isSimulation()) {
                        Pose2d bluePose = new Pose2d(7.5, 5.95, Rotation2d.k180deg);
                        Pose2d redPose = new Pose2d(10.2, 3.5, Rotation2d.kZero);
                        boolean red = DriverStation.getAlliance().get() == Alliance.Red;
                        
                        localizationSubsystem.resetPose(red ? redPose : bluePose);
                    }
                }
            ),
            new ParallelCommandGroup(
                new AutoAlignToPipe(driveSubsystem, "I").withTimeout(10),
                // 2. Once aligned, move the arm and elevator to the L2 scoring position.
                new SetScoringPositionCommand(armSubsystem, elevatorSubsystem, ScoringLevel.L2).withTimeout(1.5)
            ),
            // Score the coral withTimeout(1.5) ensures it runs for a fixed duration.
            new CoralIntakeCommand(coralIntakeSubsystem, CoralIntakeDirection.Down).withTimeout(1),
            // Go to intake position + drive to CS
            new ParallelCommandGroup(
                new AutoAlignToCS(driveSubsystem, CoralStationSide.LEFT).withTimeout(10),
                new SetScoringPositionCommand(armSubsystem, elevatorSubsystem, ScoringLevel.LOADING).withTimeout(1.5)
            ),
            
            new CoralIntakeCommand(coralIntakeSubsystem, CoralIntakeDirection.Up).withTimeout(2),
            new ParallelCommandGroup(
                new AutoAlignToPipe(driveSubsystem, "J").withTimeout(10),
                // 2. Once aligned, move the arm and elevator to the L2 scoring position.
                new SetScoringPositionCommand(armSubsystem, elevatorSubsystem, ScoringLevel.L2).withTimeout(1.5)
            ),

            // 3. Run the coral intake downwards to score the piece.
            //    withTimeout(1.5) ensures it runs for a fixed duration.
            new CoralIntakeCommand(coralIntakeSubsystem, CoralIntakeDirection.Down).withTimeout(1.5)
        );
    }
}

//be sure to align only to pipes I and J for now!
