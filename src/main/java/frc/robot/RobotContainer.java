package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.AutoL3Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.ReefState;
import frc.robot.subsystems.VisionSubsystem;
// Other imports may be needed from your existing file
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.AlgaeCommand.AlgaeDirection;
import frc.robot.commands.AlgaeIntakeCommand.AlgaeIntakeDirection;
import frc.robot.commands.CoralIntakeCommand.CoralIntakeDirection;
import frc.robot.commands.ElevatorCommand.ElevatorDirection;
import frc.robot.commands.MoveArmCommand.ArmDirection;
import frc.robot.commands.SetScoringPositionCommand.ScoringLevel;

public class RobotContainer {
  // Enum for autonomous modes
  private enum AutoMode {
    DO_NOTHING,
    L3_AUTO,
    SIMPLE_AUTO_DRIVE
  }

  // The robot's subsystems
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  // Pass the drive subsystem to vision for orientation data
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(driveSubsystem);
  // The new localization subsystem fuses their data
  private final LocalizationSubsystem localizationSubsystem = new LocalizationSubsystem(driveSubsystem, visionSubsystem);

  // Keep other subsystems as they are
  private final CANAlgaeSubsystem algaeSubsystem = new CANAlgaeSubsystem();
  private final CANElevatorSubsystem elevatorSubsystem = new CANElevatorSubsystem();
  private final CANArmSubsystem armSubsystem = new CANArmSubsystem();
  private final CANAlgaeIntakeSubsystem algaeIntakeSubsystem = new CANAlgaeIntakeSubsystem();
  private final CANCoralIntakeSubsystem coralIntakeSubsystem = new CANCoralIntakeSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  // State tracking for scoring
  private final ReefState reefState = new ReefState();

  // The autonomous commands and chooser
  private final Command autoL3Command;
  private final Command simpleAutoDriveCommand;
  private final SendableChooser<AutoMode> autoChooser = new SendableChooser<>();


  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  public RobotContainer() {
    // Instantiate the autonomous commands
    autoL3Command = new AutoL3Command(
      driveSubsystem, 
      localizationSubsystem, 
      visionSubsystem, 
      armSubsystem, 
      elevatorSubsystem, 
      coralIntakeSubsystem, 
      reefState);
    
    simpleAutoDriveCommand = new AutoCommand(driveSubsystem);

    // Configure the auto chooser
    autoChooser.setDefaultOption("Do Nothing", AutoMode.DO_NOTHING);
    autoChooser.addOption("Simple Auto (Drive Fwd)", AutoMode.SIMPLE_AUTO_DRIVE);
    autoChooser.addOption("L3 Auto", AutoMode.L3_AUTO);
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    // --- Your Existing Bindings ---
    operatorController.y().whileTrue(new ElevatorCommand(elevatorSubsystem, ElevatorDirection.Up));
    operatorController.a().whileTrue(new ElevatorCommand(elevatorSubsystem, ElevatorDirection.Down));
    operatorController.rightTrigger().whileTrue(new MoveArmCommand(armSubsystem, ArmDirection.Up));
    operatorController.leftTrigger().whileTrue(new MoveArmCommand(armSubsystem, ArmDirection.Down));
    operatorController.leftBumper().whileTrue(new CoralIntakeCommand(coralIntakeSubsystem, CoralIntakeDirection.Up));
    operatorController.rightBumper().whileTrue(new CoralIntakeCommand(coralIntakeSubsystem, CoralIntakeDirection.Down));
    operatorController.pov(0).onTrue(new SetScoringPositionCommand(armSubsystem, elevatorSubsystem, ScoringLevel.L3));
    operatorController.pov(180).onTrue(new SetScoringPositionCommand(armSubsystem, elevatorSubsystem, ScoringLevel.L2));
    // New binding for D-Pad Left to go to the loading position
    operatorController.pov(270).onTrue(new SetScoringPositionCommand(armSubsystem, elevatorSubsystem, ScoringLevel.LOADING));

    driverController.rightTrigger().onTrue(new AlgaeCommand(algaeSubsystem, AlgaeDirection.Down, algaeIntakeSubsystem));
    driverController.rightTrigger().onFalse(new AlgaeCommand(algaeSubsystem, AlgaeDirection.Up, algaeIntakeSubsystem));
    driverController.leftBumper().whileTrue(new AlgaeIntakeCommand(algaeIntakeSubsystem, AlgaeIntakeDirection.Up));
    driverController.rightBumper().whileTrue(new AlgaeIntakeCommand(algaeIntakeSubsystem, AlgaeIntakeDirection.Down));
    driverController.y().whileTrue(new ClimberClimbCommand(climberSubsystem));
    
    // --- UPDATED Auto Align Binding ---
    // The A button now triggers the new, more intelligent auto-align command
    AutoAlignCommand autoAlignCommand = new AutoAlignCommand(
        driveSubsystem, 
        localizationSubsystem, 
        visionSubsystem, 
        reefState);
        
    driverController.a().whileTrue(autoAlignCommand);

    // It's useful to have a button to mark a target as "scored"
    // For example, the B button could do this after you score.
    driverController.b().onTrue(
        autoAlignCommand.getMarkScoredCommand()
    );

     // The old 'a' button binding for climber lower is now on 'x' to avoid conflict
     driverController.x().whileTrue(new ClimberLowerCommand(climberSubsystem));
     
     // Added back the binding for the LED subsystem
     driverController.start().onTrue(new InstantCommand(ledSubsystem::cycleState, ledSubsystem));
  }

  public Command getAutonomousCommand() {
    // Get the selected autonomous mode from the chooser
    AutoMode selected = autoChooser.getSelected();
    
    // Return the corresponding command
    switch (selected) {
      case L3_AUTO:
        return autoL3Command;
      case SIMPLE_AUTO_DRIVE:
        return simpleAutoDriveCommand;
      case DO_NOTHING:
      default:
        // By default, do nothing
        return null;
    }
  }
}
