package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
  private SendableChooser<Command> pathPlannerChooser; 


  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // Slew rate limiters for smoother driving
  private final SlewRateLimiter fwdLimiter = new SlewRateLimiter(1.0);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(1.0);

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
    RobotConfig robotConfig = null; 
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
        System.err.println("CRITICAL ERROR: Failed to load PathPlanner RobotConfig from GUI settings. Path following will not work.");
        e.printStackTrace();
    }
    
    if (robotConfig != null) {
      AutoBuilder.configure(
          localizationSubsystem::getPose,
          localizationSubsystem::resetPose,
          driveSubsystem::getChassisSpeeds,
          (speeds, ff) -> driveSubsystem.setChassisSpeeds(speeds),
          new PPLTVController(0.02),
          robotConfig,
          () -> {
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
          },
          driveSubsystem
      );
      pathPlannerChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("PathPlanner Autos", pathPlannerChooser);
    }


    // Register all preset commands for use in PathPlanner
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
    NamedCommands.registerCommand("autoAlign", new AutoAlignCommand(driveSubsystem, localizationSubsystem, visionSubsystem, reefState));
    NamedCommands.registerCommand("markScored", new AutoAlignCommand(driveSubsystem, localizationSubsystem, visionSubsystem, reefState).getMarkScoredCommand());
    NamedCommands.registerCommand("cycleLed", new InstantCommand(ledSubsystem::cycleState, ledSubsystem));

    // Configure the auto chooser
    autoChooser.setDefaultOption("Do Nothing", AutoMode.DO_NOTHING);
    autoChooser.addOption("PathPlanner Auto", AutoMode.PATHPLANNER_AUTO);
    autoChooser.addOption("Simple Auto (Drive Fwd)", AutoMode.SIMPLE_AUTO_DRIVE);
    autoChooser.addOption("L3 Auto", AutoMode.L3_AUTO);
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    // Default drive command with slew rate limiting and cubic scaling
    driveSubsystem.setDefaultCommand(new RunCommand(
      () -> {
          // Get raw inputs
          double fwd = -driverController.getLeftY();
          double rot = -driverController.getRightX();

          // Apply deadband
          fwd = MathUtil.applyDeadband(fwd, OperatorConstants.CONTROLLER_DEADZONE);
          rot = MathUtil.applyDeadband(rot, OperatorConstants.CONTROLLER_DEADZONE);

          // Apply slew rate limiting
          double fwdLimited = fwdLimiter.calculate(fwd);
          double rotLimited = rotLimiter.calculate(rot);

          // Apply cubic curve for finer control
          double fwdCubic = Math.pow(fwdLimited, 3);
          double rotCubic = Math.pow(rotLimited, 3);

          driveSubsystem.arcadeDrive(fwdCubic, rotCubic);
      },
      driveSubsystem));
      
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
        // Safely get the selected auto command, returning null if the chooser wasn't created
        return (pathPlannerChooser != null) ? pathPlannerChooser.getSelected() : null;
      case DO_NOTHING:
      default:
        return null;
    }
  }

  public void simulationInit() {
      localizationSubsystem.resetPose(new Pose2d(2.0, 4.0, new Rotation2d(0)));
  }

  public void simulationPeriodic() {
      driveSubsystem.simulationPeriodic();
      // Vision has been removed from simulationPeriodic
  }
}

