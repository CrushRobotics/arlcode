package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.commands.AlgaeCommand.AlgaeDirection;
import frc.robot.commands.AlgaeIntakeCommand.AlgaeIntakeDirection;
import frc.robot.commands.CoralIntakeCommand.CoralIntakeDirection;
import frc.robot.commands.ElevatorCommand.ElevatorDirection;
import frc.robot.commands.MoveArmCommand.ArmDirection;
import frc.robot.commands.SetScoringPositionCommand.ScoringLevel;
import frc.robot.subsystems.*;

public class RobotContainer {

  // The robot's subsystems
  private final TankDriveSubsystem driveSubsystem = new TankDriveSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(driveSubsystem);
  private final LocalizationSubsystem localizationSubsystem = new LocalizationSubsystem(driveSubsystem, visionSubsystem);

  // Other subsystems
  private final CANAlgaeSubsystem algaeSubsystem = new CANAlgaeSubsystem();
  private final CANElevatorSubsystem elevatorSubsystem = new CANElevatorSubsystem();
  private final CANArmSubsystem armSubsystem = new CANArmSubsystem();
  private final CANAlgaeIntakeSubsystem algaeIntakeSubsystem = new CANAlgaeIntakeSubsystem();
  private final CANCoralIntakeSubsystem coralIntakeSubsystem = new CANCoralIntakeSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  // State tracking for scoring
  private final ReefState reefState = new ReefState();

  // The autonomous command chooser
  private SendableChooser<Command> autoChooser;

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  public RobotContainer() {
    // --- Drive Subsystem Default Command ---
    driveSubsystem.setDefaultCommand(new TeleopDriveCommand(
        driveSubsystem,
        () -> -driverController.getLeftY(),
        () -> -driverController.getRightX()
    ));

    // Default command for localization to ensure its periodic runs
    localizationSubsystem.setDefaultCommand(new RunCommand(() -> {}, localizationSubsystem));

    // Register named commands for PathPlanner
    registerNamedCommands();

    // --- PathPlanner Auto Configuration ---
    try {
        // Load the RobotConfig from the GUI settings.
        RobotConfig config = RobotConfig.fromGUISettings();

        // Configure AutoBuilder for a differential drive (Tank Drive)
        AutoBuilder.configure(
            localizationSubsystem::getPose, // Robot pose supplier
            localizationSubsystem::resetPose, // Method to reset odometry
            driveSubsystem::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds) -> driveSubsystem.setChassisSpeeds(speeds), // Method that will drive the robot
            new PPLTVController(0.02), // PPLTVController is the built in path following controller
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              var alliance = DriverStation.getAlliance();
              return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            },
            driveSubsystem // Reference to this subsystem to set requirements
        );

        // If configuration is successful, build the chooser.
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    } catch (Exception e) {
        System.err.println("CRITICAL ERROR: Failed to load PathPlanner settings. Path following will not work.");
        e.printStackTrace();
        // If it fails, create a dummy chooser that just has a "do nothing" option.
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("DO NOTHING (PathPlanner Failed)", new InstantCommand());
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    configureBindings();
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("scoreL3", new SetScoringPositionCommand(armSubsystem, elevatorSubsystem, ScoringLevel.L3));
    NamedCommands.registerCommand("scoreL2", new SetScoringPositionCommand(armSubsystem, elevatorSubsystem, ScoringLevel.L2));
    NamedCommands.registerCommand("scoreLoading", new SetScoringPositionCommand(armSubsystem, elevatorSubsystem, ScoringLevel.LOADING));
    NamedCommands.registerCommand("outtakeCoral", new CoralIntakeCommand(coralIntakeSubsystem, CoralIntakeDirection.Down).withTimeout(1.0));
    NamedCommands.registerCommand("intakeCoral", new CoralIntakeCommand(coralIntakeSubsystem, CoralIntakeDirection.Up).withTimeout(1.0));
    NamedCommands.registerCommand("elevatorUp", new ElevatorCommand(elevatorSubsystem, ElevatorDirection.Up));
    NamedCommands.registerCommand("elevatorDown", new ElevatorCommand(elevatorSubsystem, ElevatorDirection.Down));
    NamedCommands.registerCommand("armUp", new MoveArmCommand(armSubsystem, ArmDirection.Up));
    NamedCommands.registerCommand("armDown", new MoveArmCommand(armSubsystem, ArmDirection.Down));
    NamedCommands.registerCommand("algaeDown", new AlgaeCommand(algaeSubsystem, AlgaeDirection.Down, algaeIntakeSubsystem));
    NamedCommands.registerCommand("algaeUp", new AlgaeCommand(algaeSubsystem, AlgaeDirection.Up, algaeIntakeSubsystem));
    NamedCommands.registerCommand("intakeAlgae", new AlgaeIntakeCommand(algaeIntakeSubsystem, AlgaeIntakeDirection.Up));
    NamedCommands.registerCommand("outtakeAlgae", new AlgaeIntakeCommand(algaeIntakeSubsystem, AlgaeIntakeDirection.Down));
    NamedCommands.registerCommand("climb", new ClimberClimbCommand(climberSubsystem));
    NamedCommands.registerCommand("autoAlign", new AutoAlignCommand(driveSubsystem, reefState));
    NamedCommands.registerCommand("cycleLed", new InstantCommand(ledSubsystem::cycleState, ledSubsystem));
  }

  private void configureBindings() {
    // --- Operator Bindings ---
    operatorController.y().whileTrue(new ElevatorCommand(elevatorSubsystem, ElevatorDirection.Up));
    operatorController.a().whileTrue(new ElevatorCommand(elevatorSubsystem, ElevatorDirection.Down));
    operatorController.rightTrigger().whileTrue(new MoveArmCommand(armSubsystem, ArmDirection.Up));
    operatorController.leftTrigger().whileTrue(new MoveArmCommand(armSubsystem, ArmDirection.Down));
    operatorController.leftBumper().whileTrue(new CoralIntakeCommand(coralIntakeSubsystem, CoralIntakeDirection.Up));
    operatorController.rightBumper().whileTrue(new CoralIntakeCommand(coralIntakeSubsystem, CoralIntakeDirection.Down));
    operatorController.pov(0).onTrue(new SetScoringPositionCommand(armSubsystem, elevatorSubsystem, ScoringLevel.L3));
    operatorController.pov(180).onTrue(new SetScoringPositionCommand(armSubsystem, elevatorSubsystem, ScoringLevel.L2));
    operatorController.pov(270).onTrue(new SetScoringPositionCommand(armSubsystem, elevatorSubsystem, ScoringLevel.LOADING));

    // --- Driver Bindings ---
    driverController.rightTrigger().onTrue(new AlgaeCommand(algaeSubsystem, AlgaeDirection.Down, algaeIntakeSubsystem));
    driverController.rightTrigger().onFalse(new AlgaeCommand(algaeSubsystem, AlgaeDirection.Up, algaeIntakeSubsystem));
    driverController.leftBumper().whileTrue(new AlgaeIntakeCommand(algaeIntakeSubsystem, AlgaeIntakeDirection.Up));
    driverController.rightBumper().whileTrue(new AlgaeIntakeCommand(algaeIntakeSubsystem, AlgaeIntakeDirection.Down));
    driverController.y().whileTrue(new ClimberClimbCommand(climberSubsystem));

    // --- Trajectory Auto-Align Binding ---
    AutoAlignCommand autoAlignCommand = new AutoAlignCommand(driveSubsystem, reefState);
    driverController.a().whileTrue(autoAlignCommand);

    // Button to mark a target as "scored"
    driverController.b().onTrue(autoAlignCommand.getMarkScoredCommand());

    // LED subsystem binding
    driverController.x().onTrue(new InstantCommand(ledSubsystem::cycleState, ledSubsystem));

    // Reset gyro/odometry
    driverController.start().onTrue(new InstantCommand(() -> driveSubsystem.resetPose(new Pose2d())));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**
   * This method is called once when simulation starts.
   */
  public void simulationInit() {
      driveSubsystem.resetPose(new Pose2d(2.0, 4.0, new Rotation2d(0)));
  }

  /**
   * This method is called periodically during simulation.
   */
  public void simulationPeriodic() {
      // Run the drivetrain simulation update
      driveSubsystem.simulationPeriodic();

      // THE FIX: The simulated vision logic has been removed from this method.
      // The circular feedback loop between the pose estimator and the simulated
      // vision data was causing the robot to spin uncontrollably. By removing it,
      // the simulator will now rely only on the stable drive odometry (simulated
      // encoders and gyro), which will stop the spinning.
  }
}

