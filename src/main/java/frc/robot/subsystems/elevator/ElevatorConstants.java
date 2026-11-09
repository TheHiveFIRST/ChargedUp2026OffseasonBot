package frc.robot.subsystems.elevator;

public final class ElevatorConstants {
   public static final int ELEVATOR_LEADER_ID = 12;
    public static final int ELEVATOR_FOLLOWER_ID = 13;

    // PID Gains
    public static final double ELEVATOR_KP = 2;
    public static final double ELEVATOR_KI = 0.0;
    public static final double ELEVATOR_KD = 0.0;
    

    public static final double Elevator_OUT_MID = 0.73; // Need tuning
    public static final double Elevator_OUT_HIGH = 0.73; // Need tuning
    public static final double Elevator_IN_GROUND = 0.24; // Need tuning
    public static final double Elevator_IN_HUMAN = 0.24; // Need tuning

    public static final double POSITION_TOLERANCE = 0.1;
  }
