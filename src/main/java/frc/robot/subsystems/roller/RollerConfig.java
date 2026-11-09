package frc.robot.subsystems.roller;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class RollerConfig {
  // Intake motor config
  public static final SparkFlexConfig rollerMotorConfig = new SparkFlexConfig();

  static {
    rollerMotorConfig.smartCurrentLimit(80); //set current limit to 80A
  }
}
