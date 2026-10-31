

package frc.robot.subsystems.elevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.roller.RollerConstants;


public class Elevator extends SubsystemBase {
  private final SparkMax mElevatorLeader;
  private final SparkMax mElevatorFollower;
  private final AbsoluteEncoder mElevatorFollowerEncoder;
  private final AbsoluteEncoder mElevatorLeaderEncoder;
  private final PIDController mElevatorLeaderPID;
  private final PIDController mElevatorFollowerPID;

  public double mElevatorOutputPower = 0;
  
  private double mCurrentTarget = ElevatorConstants.Elevator_IN;
  private boolean mIntakeGroundSecondStage = false;

  public Elevator() {
    mElevatorLeader = new SparkMax(ElevatorConstants.ELEVATOR_LEADER_ID, MotorType.kBrushless);
    mElevatorFollower =
        new SparkMax(ElevatorConstants.ELEVATOR_FOLLOWER_ID, MotorType.kBrushless);

    // Applying configuration
    mElevatorLeader.configure(
        ElevatorConfig.elevatorLeaderConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    mElevatorFollower.configure(
        ElevatorConfig.elevatorFollowerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    mElevatorFollowerEncoder = mElevatorFollower.getAbsoluteEncoder();
    mElevatorLeaderEncoder = mElevatorLeader.getAbsoluteEncoder();

    // PID constants for arm
    mElevatorFollowerPID =
        new PIDController(
            ElevatorConstants.ELEVATOR_KP,
            ElevatorConstants.ELEVATOR_KI,
            ElevatorConstants.ELEVATOR_KD);

    mElevatorLeaderPID =
        new PIDController(
            ElevatorConstants.ELEVATOR_KP,
            ElevatorConstants.ELEVATOR_KI,
            ElevatorConstants.ELEVATOR_KD);

    mElevatorLeaderPID.setTolerance(ElevatorConstants.POSITION_TOLERANCE);
    mElevatorFollowerPID.setTolerance(ElevatorConstants.POSITION_TOLERANCE);
  }
  // Methods
  /**
   * Set the desired Elevator position and drive the motors with the follower PID.
   *
   * @param position desired Elevator position (encoder units)
   */
  public void setTargetArmPosition(double position) {
    mCurrentTarget = position;
    // Use the field mElevatorOutputPower (avoid shadowing a local variable)
    mElevatorOutputPower =
        mElevatorFollowerPID.calculate(mElevatorFollowerEncoder.getPosition(), mCurrentTarget);
    mElevatorFollower.set(mElevatorOutputPower);
    mElevatorLeader.set(mElevatorOutputPower);
  }

  /**
   * Set arm Elevator motor power directly (open-loop).
   *
   * @param ElevatorPower motor power in range [-1.0, 1.0]
   */
  public void setArmPower(double ElevatorPower) {
    mElevatorLeader.set(ElevatorPower);
    mElevatorFollower.set(ElevatorPower);
  }

  /**
   * Get the follower absolute encoder position.
   *
   * @return encoder position
   */
  public double getEncoderValue() {
    return mElevatorFollowerEncoder.getPosition();
  }

  /**
   * Query whether the leader PID controller is at its setpoint.
   *
   * @return true if at setpoint
   */
  public boolean isAtTarget() {
    return mElevatorLeaderPID.atSetpoint();
  }

  /** Stop the arm motors and hold current position by updating the target. */
  public void stopArm() {
    mElevatorLeader.set(0);
    mCurrentTarget = mElevatorLeaderEncoder.getPosition(); // Hold current spot
  }

  /** Increase the internal KP tuning value by the configured increment. */
  

  // Commands
  public Command IntakeToPositionCommand(double position) {
    return run(
        () -> {
          setTargetArmPosition(position);
        });
  }

  public Command runIntakeElevatorGroundCommand() {
    return run(
        () -> {
          if (getEncoderValue() < 0.7) {
            setTargetArmPosition(ElevatorConstants.Elevator_OUT);
          } else {
            setArmPower(0);
          }
        });
  }

  public Command runIntakeElevatorBumpCommand() {
    return run(
        () -> {
          setTargetArmPosition(ElevatorConstants.Elevator_BUMP);
        });
  }

  public Command runIntakeElevatorInCommand() {
    return run(
        () -> {
          setTargetArmPosition(ElevatorConstants.Elevator_IN);
        });
  }

  public Command IntakeSlightlyUpCommand() {
    return run(
        () -> {
          setTargetArmPosition(ElevatorConstants.Elevator_AGITATE);
        });
  }

  public Command stopElevatorCommand() {
    return run(
        () -> {
          setArmPower(0.0);
        });
  }

  public Command runElevatorCommand() {
    return run(
        () -> {
          setArmPower(-0.2);
        });
  }

  public Command toggleArmStageCommand() {
    return new InstantCommand(() -> mIntakeGroundSecondStage = !mIntakeGroundSecondStage);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm/Elevator Encoder Value", getEncoderValue());
    SmartDashboard.putNumber("Arm/Target Position", mCurrentTarget);
  }
}
