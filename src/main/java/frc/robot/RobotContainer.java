// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeCommand;
import frc.robot.commands.AlgaeCommand.AlgaeDirection;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeIntakeCommand.AlgaeIntakeDirection;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.ClimberClimbCommand;
import frc.robot.commands.ClimberLowerCommand;
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
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  // The robot's subsystems
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final CANAlgaeSubsystem algaeSubsystem = new CANAlgaeSubsystem();
  private final CANElevatorSubsystem elevatorSubsystem = new CANElevatorSubsystem();
  private final CANArmSubsystem armSubsystem = new CANArmSubsystem();
  private final CANAlgaeIntakeSubsystem algaeIntakeSubsystem = new CANAlgaeIntakeSubsystem();
  private final CANCoralIntakeSubsystem coralIntakeSubsystem = new CANCoralIntakeSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.DRIVER_CONTROLLER_PORT);

  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);
  
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  
  // Hardware for the autonomous command
  public final TalonFX leftLeader = new TalonFX(8);
  public final TalonFX rightLeader = new TalonFX(7);
  
  private final Timer timer = new Timer();

  public RobotContainer() {
    configureBindings();
    autoChooser.setDefaultOption("Autonomous", new AutoCommand(driveSubsystem));
  }

  private void configureBindings() {
    // Operator Controller Bindings
    operatorController.y().whileTrue(new ElevatorCommand(elevatorSubsystem, ElevatorDirection.Up));
    operatorController.a().whileTrue(new ElevatorCommand(elevatorSubsystem, ElevatorDirection.Down));
    operatorController.rightTrigger().whileTrue(new MoveArmCommand(armSubsystem, ArmDirection.Up));
    operatorController.leftTrigger().whileTrue(new MoveArmCommand(armSubsystem, ArmDirection.Down));
    operatorController.leftBumper().whileTrue(new CoralIntakeCommand(coralIntakeSubsystem, CoralIntakeDirection.Up));
    operatorController.rightBumper().whileTrue(new CoralIntakeCommand(coralIntakeSubsystem, CoralIntakeDirection.Down));

    // New Bindings for Scoring Presets using DPad
    operatorController.pov(0).onTrue(new SetScoringPositionCommand(armSubsystem, elevatorSubsystem, ScoringLevel.L3)); // DPad Up for L3
    operatorController.pov(180).onTrue(new SetScoringPositionCommand(armSubsystem, elevatorSubsystem, ScoringLevel.L2)); // DPad Down for L2


    // Driver Controller Bindings
    driverController.rightTrigger().onTrue(new AlgaeCommand(algaeSubsystem, AlgaeDirection.Down, algaeIntakeSubsystem));
    driverController.rightTrigger().onFalse(new AlgaeCommand(algaeSubsystem, AlgaeDirection.Up, algaeIntakeSubsystem));
    driverController.leftBumper().whileTrue(new AlgaeIntakeCommand(algaeIntakeSubsystem, AlgaeIntakeDirection.Up));
    driverController.rightBumper().whileTrue(new AlgaeIntakeCommand(algaeIntakeSubsystem, AlgaeIntakeDirection.Down));
    
    // Bind the X button to an inline InstantCommand that cycles the LED state.
    driverController.x().onTrue(new InstantCommand(ledSubsystem::cycleState, ledSubsystem));
    
    // Bind the A button to the AutoAlignCommand
    driverController.a().whileTrue(new AutoAlignCommand(driveSubsystem, visionSubsystem));

    // Bind the Y and A buttons for the climber
    driverController.y().whileTrue(new ClimberClimbCommand(climberSubsystem));
    driverController.a().whileTrue(new ClimberLowerCommand(climberSubsystem));
  }

  public Command getAutonomousCommand() {
    return new FunctionalCommand(
      // initialize() - Runs once when the command starts
      () -> {
        timer.reset();
        timer.start();
      },
      // execute() - Runs repeatedly until the command ends
      () -> {
        // Drive forward at a constant speed
        leftLeader.setControl(new DutyCycleOut(0.25));
        rightLeader.setControl(new DutyCycleOut(0.25));
        // **FIXED**: Run the intake motor simultaneously
        coralIntakeSubsystem.coralIntakeMotor.set(0.2);
      },
      // end() - Runs once when the command ends or is interrupted
      (interrupted) -> {
        // Stop all motors
        rightLeader.setControl(new DutyCycleOut(0));
        leftLeader.setControl(new DutyCycleOut(0));
        coralIntakeSubsystem.stop();
        timer.stop();
      },
      // isFinished() - Returns true when the command should end
      () -> timer.get() > 3,
      // **FIXED**: Added coralIntakeSubsystem as a requirement
      driveSubsystem, coralIntakeSubsystem
    );
  }
}
