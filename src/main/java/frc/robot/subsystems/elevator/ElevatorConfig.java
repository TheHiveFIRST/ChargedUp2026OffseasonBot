package frc.robot.subsystems.elevator;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class ElevatorConfig {
  public static final SparkMaxConfig elevatorLeaderConfig = new SparkMaxConfig();
  public static final SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();
  

  static {
    // Leader configuration
    elevatorLeaderConfig.smartCurrentLimit(50).idleMode(IdleMode.kCoast);
    // Follower configuration - set to follow Leader and invert
    elevatorFollowerConfig.inverted(true).smartCurrentLimit(50).idleMode(IdleMode.kCoast);

    // Feeder/Kicker configuration
    
  }
}
