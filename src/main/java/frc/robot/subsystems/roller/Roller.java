package frc.robot.subsystems.roller;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Roller extends SubsystemBase {
  private final SparkFlex mIntakeMotor;

  // TODO: ADD comments
  public Roller() {
    mIntakeMotor = new SparkFlex(RollerConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

    mIntakeMotor.configure(
        RollerConfig.intakeMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  // Methods
  public void stopIntake() {
    runIntake(0.0);
  }

  public void runIntake(double speed) {
    mIntakeMotor.set(speed);
  }

  // Command factories
  public Command runIntakeForwardCommand() {
    return Commands.run(() -> runIntake(RollerConstants.INTAKE_SPEED), this);
  }

  public Command runIntakeSlowCommand() {
    return Commands.run(() -> runIntake(RollerConstants.SLOW_INTAKE_SPEED), this);
  }

  public Command runOuttakeCommand() {
    return Commands.run(() -> runIntake(RollerConstants.OUTTAKE_SPEED), this);
  }

  public Command stopIntakeCommand() {
    return Commands.runOnce(this::stopIntake, this);
  }

  @Override
  public void periodic() {
    // EX: SmartDashboard.putNumber("Intake/AppliedVolts", mIntakeMotor.getAppliedOutput());
  }
}
