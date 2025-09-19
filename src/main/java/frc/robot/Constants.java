package frc.robot;

public final class Constants {

  // Operator Interface
  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  // Drivetrain (Phoenix Pro) - IDs from previous configuration
  public static final class DriveConstants {
    public static final int LEFT_LEADER_ID = 8;
    public static final int LEFT_FOLLOWER_ID = 9;
    public static final int RIGHT_LEADER_ID = 7;
    public static final int RIGHT_FOLLOWER_ID = 6;
  }
  
  // Elevator
  public static final class ElevatorConstants {
    public static final int ELEVATOR_LEADER_ID = 2;
    public static final int ELEVATOR_FOLLOWER_ID = 4;
  }

  // Arm / Coral Arm
  public static final class ArmConstants {
    public static final int ARM_MOTOR_ID = 3; // Coral Arm Motor
    // The following IDs are from previous versions of the code. Please verify them.
    public static final int ARM_PIVOT_ID = 14; 
    public static final int ARM_SHOOTER_ID = 15;
  }

  // Algae Intake
  public static final class AlgaeIntakeConstants {
    public static final int ALGAE_INTAKE_ID = 11; // Algae Intake motor
  }
  
  // Algae Subsystem (Arm)
  public static class AlgaeConstants {
    public static final int ALGAE_MOTOR_ID = 21; // Algae Intake Arm
  }

  // Coral Intake
  public static class CoralIntakeConstants {
    public static final int CORAL_INTAKE_ID = 16;
  }

  // Climber
  public static class ClimberConstants{
    public static final int LEFT_CLIMBER_ID = 27;
    public static final int RIGHT_CLIMBER_ID = 24; // This was the previous ID, please verify
    public static final double MIN_BOUND = 0.1;
    public static final double MAX_BOUND = 85;
  }

  // PID Configurations
  public static class PIDConstants{
    public static final double P = 0.1; 
    public static final double I = 0;
    public static final double D = 0;
    public static final double G = 9.8;
  }

  public static class PIDElevatorConstants{
    public static final double P = 0.15;
    public static final double I = 0;
    public static final double D = 0;
    public static final double G = 23;
  }
  
}

