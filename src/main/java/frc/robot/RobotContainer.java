package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.config.ReplanningConfig; // Commented out due to unresolved import
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeCommand;
import frc.robot.commands.AlgaeCommand.AlgaeDirection;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeIntakeCommand.AlgaeIntakeDirection;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.AutoL3Command;
import frc.robot.commands.ClimberClimbCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralIntakeCommand.CoralIntakeDirection;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorCommand.ElevatorDirection;
import frc.robot.commands.MoveArmCommand;
import frc.robot.commands.MoveArmCommand.ArmDirection;
import frc.robot.commands.SetScoringPositionCommand;
import frc.robot.commands.SetScoringPositionCommand.ScoringLevel;
import frc.robot.subsystems.CANAlgaeIntakeSubsystem;
import frc.robot.subsystems.CANAlgaeSubsystem;
import frc.robot.subsystems.CANArmSubsystem;
import frc.robot.subsystems.CANCoralIntakeSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANElevatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.ReefState;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  // Enum for autonomous modes
  private enum AutoMode {
    DO_NOTHING,
    L3_AUTO,
    SIMPLE_AUTO_DRIVE,
    PATHPLANNER_AUTO
  }

  // The robot's subsystems
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(driveSubsystem);
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
  private SendableChooser<Command> pathPlannerChooser; // Initialized as null


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

    // --- PATHPLANNER SETUP ---
    // The following PathPlanner configuration is commented out because the
    // ReplanningConfig class cannot be found. This is likely due to a
    // project dependency issue. To re-enable PathPlanner, you will need to
    // resolve the dependency issue, likely by cleaning the project and
    // refreshing the vendor libraries.

    /*
    AutoBuilder.configureRamsete(
        localizationSubsystem::getPose, // Robot pose supplier
        localizationSubsystem::resetPose, // Method to reset odometry
        driveSubsystem::getChassisSpeeds, // Current chassis speeds supplier
        driveSubsystem::setChassisSpeeds, // Method that will drive the robot
        new ReplanningConfig(), // Default path replanning config
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        driveSubsystem // Drive subsystem requirement
    );

    // Register named commands
    NamedCommands.registerCommand("exampleCommand", new InstantCommand(() -> System.out.println("Ran example command!")));
    NamedCommands.registerCommand("scoreL3", new SetScoringPositionCommand(armSubsystem, elevatorSubsystem, ScoringLevel.L3));
    NamedCommands.registerCommand("outtakeCoral", new CoralIntakeCommand(coralIntakeSubsystem, CoralIntakeDirection.Down).withTimeout(1.0));

    // Create a chooser for PathPlanner paths
    pathPlannerChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("PathPlanner Chooser", pathPlannerChooser);
    */

    // Configure the auto chooser
    autoChooser.setDefaultOption("Do Nothing", AutoMode.DO_NOTHING);
    // autoChooser.addOption("PathPlanner Auto", AutoMode.PATHPLANNER_AUTO); // Temporarily disabled
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
    operatorController.pov(270).onTrue(new SetScoringPositionCommand(armSubsystem, elevatorSubsystem, ScoringLevel.LOADING));

    driverController.rightTrigger().onTrue(new AlgaeCommand(algaeSubsystem, AlgaeDirection.Down, algaeIntakeSubsystem));
    driverController.rightTrigger().onFalse(new AlgaeCommand(algaeSubsystem, AlgaeDirection.Up, algaeIntakeSubsystem));
    driverController.leftBumper().whileTrue(new AlgaeIntakeCommand(algaeIntakeSubsystem, AlgaeIntakeDirection.Up));
    driverController.rightBumper().whileTrue(new AlgaeIntakeCommand(algaeIntakeSubsystem, AlgaeIntakeDirection.Down));
    driverController.y().whileTrue(new ClimberClimbCommand(climberSubsystem));

    // --- UPDATED Auto Align Binding ---
    AutoAlignCommand autoAlignCommand = new AutoAlignCommand(
        driveSubsystem, 
        localizationSubsystem, 
        visionSubsystem, 
        reefState);
        
    driverController.a().whileTrue(autoAlignCommand);

    // Button to mark a target as "scored"
    driverController.b().onTrue(
        autoAlignCommand.getMarkScoredCommand()
    );

     // LED subsystem binding
     driverController.x().onTrue(new InstantCommand(ledSubsystem::cycleState, ledSubsystem));
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
      case PATHPLANNER_AUTO:
        // Return null if PathPlanner is disabled, otherwise return the selected path
        return (pathPlannerChooser != null) ? pathPlannerChooser.getSelected() : null;
      case DO_NOTHING:
      default:
        return null;
    }
  }
}

