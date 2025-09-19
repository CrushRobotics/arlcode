package frc.robot;

public final class Constants {
  public static final class DriveConstants {
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
  }

  public static final class RollerConstants {
    public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
    public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
    public static final double ROLLER_EJECT_VALUE = 0.44;
  }

  public static final class ElevatorConstants {
    public static final int ELEVATOR_LEADER_ID = 2;
    public static final int ELEVATOR_FOLLOWER_ID = 4;
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }
  public static final class ArmConstants {
    public static final double MIN_BOUND = -13;
    public static final double MAX_BOUND = -.5;
    public static final double SHOOTING_POSITION = -7.1;
    public static final double SOURCE_POSITION = -3.57;
    public static final int ARM_MOTOR_ID = 15; // Arm motor
    public static final int ARM_GEAR_ID = 14;  // motor to pivot arm 
  }

  // Added AlgaeIntakeConstants to store the CAN ID
  public static class AlgaeIntakeConstants {
    public static final int ALGAE_INTAKE_ID = 4;
  }

  public static class ClimberConstants{
    public static final double MIN_BOUND = 0.1;
    public static final double MAX_BOUND = 85;
  }
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

