package frc.robot.subsystems.roller;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Roller extends SubsystemBase {
  private final SparkFlex mRollerMotor;

  // TODO: ADD comments
  public Roller() {
    // Creats a new SparkFlex motor controller for the roller motor
    mRollerMotor = new SparkFlex(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushless); 

    // Applies the configuration to the roller motor
    mRollerMotor.configure(
        RollerConfig.rollerMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  // Methods
  public void stopIntake() {
    runIntake(0.0);
  }

  // Runs the intake at a target speed
  public void runIntake(double speed) {
    mRollerMotor.set(speed);
  }

  // Command factories

  // Runs the roller at full speed to intake a game piece
  public Command runIntakeForwardCommand() {
    return Commands.run(() -> runIntake(RollerConstants.INTAKE_SPEED), this);
  }

  // Runs the roller at a slower speed to intake a game piece
  public Command runIntakeSlowCommand() {
    return Commands.run(() -> runIntake(RollerConstants.SLOW_INTAKE_SPEED), this);
  }

  // Runs the roller in reverse to outtake a game piece
  public Command runOuttakeCommand() {
    return Commands.run(() -> runIntake(RollerConstants.OUTTAKE_SPEED), this);
  }

  public Command stopIntakeCommand() {
    return Commands.runOnce(this::stopIntake, this);
  }

  @Override
  public void periodic() {
    // EX: SmartDashboard.putNumber("Intake/AppliedVolts", mRollerMotor.getAppliedOutput());
  }
}
