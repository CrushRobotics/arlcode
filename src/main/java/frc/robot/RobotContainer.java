package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeCommand;
import frc.robot.commands.AlgaeCommand.AlgaeDirection;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AlgaeIntakeCommand.AlgaeIntakeDirection;
// import frc.robot.commands.AutoAlign; // No longer used
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.AutoAlignCommand.AlignMode;
import frc.robot.commands.ClimberClimbCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralIntakeCommand.CoralIntakeDirection;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorCommand.ElevatorDirection;
import frc.robot.commands.MoveArmCommand;
import frc.robot.commands.MoveArmCommand.ArmDirection;
import frc.robot.commands.RunSysIDTests;
import frc.robot.commands.SetScoringPositionCommand;
import frc.robot.commands.SetScoringPositionCommand.ScoringLevel;
import frc.robot.commands.autos.AutoCommand;
import frc.robot.commands.autos.AutoL2Command;
import frc.robot.commands.autos.AutoL3Command;
import frc.robot.commands.autos.L2SimpleCommand;
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
    AUTO_ALIGN_L2_AUTO,
    L2_SIMPLE_AUTO,
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
  // These are now instantiated in getAutonomousCommand()
  private final SendableChooser<AutoMode> autoChooser = new SendableChooser<>();
  private SendableChooser<Command> pathPlannerChooser;


  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // Slew rate limiters for smoother driving
  private final SlewRateLimiter fwdLimiter = new SlewRateLimiter(3.0);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3.0);

  public RobotContainer() {
    driveSubsystem.setLocalizationSubsystem(localizationSubsystem);

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
          localizationSubsystem::resetPose, // Pose Estimator reset is done here
          driveSubsystem::getChassisSpeeds,
          driveSubsystem::setChassisSpeeds, // Corrected method reference
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
    NamedCommands.registerCommand("autoAlign", new AutoAlignCommand(driveSubsystem, localizationSubsystem, visionSubsystem, reefState, AlignMode.SCORING));

    // Updated "markScored" command
    NamedCommands.registerCommand("markScored", new InstantCommand(() -> {
        String lastTarget = reefState.getLastTargetedPipe();
        if (lastTarget != null) {
            reefState.markScored(lastTarget);
            System.out.println("[NamedCmd] Marked " + lastTarget + " as scored.");
        } else {
            System.out.println("[NamedCmd] Mark Scored: No target was locked.");
        }
    }, reefState));

    NamedCommands.registerCommand("cycleLed", new InstantCommand(ledSubsystem::cycleState, ledSubsystem));

     // Configure the auto chooser
     autoChooser.setDefaultOption("Do Nothing", AutoMode.DO_NOTHING);
     autoChooser.addOption("Auto Align and Score L2", AutoMode.AUTO_ALIGN_L2_AUTO);
     autoChooser.addOption("L2 Simple", AutoMode.L2_SIMPLE_AUTO);
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
          double fwdLimited = fwd;
          double rotLimited = rot;

          // Apply cubic curve for finer control
          double fwdCubic = Math.copySign(fwdLimited * fwdLimited * fwdLimited, fwdLimited);
          double rotCubic = Math.copySign(rotLimited * rotLimited * rotLimited, rotLimited);

          driveSubsystem.arcadeDrive(fwdCubic * DriveConstants.MAX_SPEED_MPS, rotCubic * DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC);
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

    // Bind the SysId command to the 'start' button
    driverController.start().whileTrue(new RunSysIDTests(driveSubsystem));

    // --- UPDATED Auto Align Binding ---
    // Use .onTrue() to start the sequence. It will find the best target when pressed
    // and execute the full sequence (drive to approach, then drive to final).
    driverController.a().whileTrue(
      new AutoAlign(driveSubsystem)
    );

    // Button to mark the *last targeted* pipe as scored
    driverController.b().onTrue(
        new InstantCommand(() -> {
            String lastTarget = reefState.getLastTargetedPipe();
            if (lastTarget != null) {
                reefState.markScored(lastTarget);
                // System.out.println("[Button B] Marked " + lastTarget + " as scored.");
            } else {
                System.out.println("[Button B] Mark Scored: No target was locked.");
            }
        }, reefState)
    );

     // LED subsystem binding
     driverController.x().onTrue(new InstantCommand(ledSubsystem::cycleState, ledSubsystem));

     // --- NEW: Manual Reset Bindings ---
     // Reset Gyro to 0 (e.g., Back button)
     driverController.back().onTrue(new InstantCommand(this::resetGyro).ignoringDisable(true));
     // Reset Pose and Gyro using Limelight (e.g., Start button + Back button combo? Reusing Start for now)
     // driverController.start().onTrue(new InstantCommand(this::resetOdometryAndGyroToLimelight).ignoringDisable(true));
     // Since Start is used by SysId, let's use a different trigger, maybe Y button + Back?
     // For simplicity, let's just add another command to the Back button press:
     driverController.back().onTrue(new InstantCommand(this::resetOdometryAndGyroToLimelight).ignoringDisable(true));

  }

  /**
   * Resets the gyroscope to zero.
   */
  public void resetGyro() {
    // System.out.println("Resetting Gyro to 0...");
    driveSubsystem.getPigeon().reset();
    // Also reset the pose estimator's rotation to align with the new gyro reading
    localizationSubsystem.resetPose(
        new edu.wpi.first.math.geometry.Pose2d(
            localizationSubsystem.getPose().getTranslation(),
            driveSubsystem.getRotation2d() // Use the newly reset gyro angle
        )
    );
    // System.out.println("Gyro reset complete.");
  }

  /**
   * Attempts to reset BOTH the pose estimator AND the NavX gyro angle
   * using the pose estimate from the Limelight cameras.
   * Prefers limelight-right, falls back to limelight-left.
   * This should ideally be called when the robot is stationary and has a clear view of tags.
   */
  public void resetOdometryAndGyroToLimelight() {
    // System.out.println("Attempting to reset Odometry and Gyro to Limelight...");
    // Prefer right limelight, fallback to left
    LimelightHelpers.PoseEstimate limelightPoseEstimate = visionSubsystem.getPoseEstimate("limelight-right");
    String source = "limelight-right";

    if (limelightPoseEstimate == null || !LimelightHelpers.validPoseEstimate(limelightPoseEstimate)) {
        limelightPoseEstimate = visionSubsystem.getPoseEstimate("limelight-left");
        source = "limelight-left";
    }

    if (limelightPoseEstimate != null && LimelightHelpers.validPoseEstimate(limelightPoseEstimate)) {
        // System.out.println("Valid Pose Estimate found from " + source + ": " + limelightPoseEstimate.pose);

        // 1. Reset the Pose Estimator
        localizationSubsystem.resetPose(limelightPoseEstimate.pose);
        // System.out.println("Pose Estimator reset to: " + limelightPoseEstimate.pose);

        // 2. Reset the NavX Gyro Yaw
        driveSubsystem.getPigeon().setYaw(limelightPoseEstimate.pose.getRotation());
        // System.out.println("NavX Yaw reset to: " + limelightPoseEstimate.pose.getRotation().getDegrees());

    } else {
        System.err.println("Reset Odometry/Gyro to Limelight FAILED: No valid pose estimate found from either camera.");
    }
}


  public Command getAutonomousCommand() {
    if (RobotBase.isSimulation()) {
      return new AutoL2Command(driveSubsystem, localizationSubsystem, visionSubsystem, reefState, armSubsystem, elevatorSubsystem, coralIntakeSubsystem);
    }
    // Get the selected autonomous mode from the chooser
    AutoMode selected = autoChooser.getSelected();

    // Reset ReefState at the start of auto
    reefState.clear();

    // Return the corresponding command, instantiating it now
    switch (selected) {
      case AUTO_ALIGN_L2_AUTO:
        return new AutoL2Command(
            driveSubsystem,
            localizationSubsystem,
            visionSubsystem,
            reefState,
            armSubsystem,
            elevatorSubsystem,
            coralIntakeSubsystem);
      case L2_SIMPLE_AUTO:
        return new L2SimpleCommand(
            driveSubsystem,
            armSubsystem,
            elevatorSubsystem,
            coralIntakeSubsystem);
      case L3_AUTO:
        return new AutoL3Command(
            driveSubsystem,
            localizationSubsystem,
            visionSubsystem,
            armSubsystem,
            elevatorSubsystem,
            coralIntakeSubsystem,
            reefState);
      case SIMPLE_AUTO_DRIVE:
        return new AutoCommand(driveSubsystem);
      case PATHPLANNER_AUTO:
        return (pathPlannerChooser != null) ? pathPlannerChooser.getSelected() : Commands.none();
      case DO_NOTHING:
      default:
        return Commands.none();
    }
  }

}

