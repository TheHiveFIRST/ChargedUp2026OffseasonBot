

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
  private final AbsoluteEncoder mPivotLeaderEncoder;
  private final PIDController mPivotLeaderPID;
  private final PIDController mElevatorFollowerPID;

  public double mPivotOutputPower = 0;
  
  private double mCurrentTarget = ElevatorConstants.PIVOT_IN;
  private boolean mIntakeGroundSecondStage = false;

  public Elevator() {
    mElevatorLeader = new SparkMax(ElevatorConstants.ARM_LEADER_ID, MotorType.kBrushless);
    mElevatorFollower =
        new SparkMax(ElevatorConstants.ARM_FOLLOWER_ID, MotorType.kBrushless);

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
    mPivotLeaderEncoder = mElevatorLeader.getAbsoluteEncoder();

    // PID constants for arm
    mElevatorFollowerPID =
        new PIDController(
            ElevatorConstants.ELEVATOR_KP,
            ElevatorConstants.ELEVATOR_KI,
            ElevatorConstants.ELEVATOR_KD);

    mPivotLeaderPID =
        new PIDController(
            ElevatorConstants.ELEVATOR_KP,
            ElevatorConstants.ELEVATOR_KI,
            ElevatorConstants.ELEVATOR_KD);

    mPivotLeaderPID.setTolerance(ElevatorConstants.POSITION_TOLERANCE);
    mElevatorFollowerPID.setTolerance(ElevatorConstants.POSITION_TOLERANCE);
  }
  // Methods
  /**
   * Set the desired pivot position and drive the motors with the follower PID.
   *
   * @param position desired pivot position (encoder units)
   */
  public void setTargetArmPosition(double position) {
    mCurrentTarget = position;
    // Use the field mPivotOutputPower (avoid shadowing a local variable)
    mPivotOutputPower =
        mElevatorFollowerPID.calculate(mElevatorFollowerEncoder.getPosition(), mCurrentTarget);
    mElevatorFollower.set(mPivotOutputPower);
    mElevatorLeader.set(mPivotOutputPower);
  }

  /**
   * Set arm pivot motor power directly (open-loop).
   *
   * @param pivotPower motor power in range [-1.0, 1.0]
   */
  public void setArmPower(double pivotPower) {
    mElevatorLeader.set(pivotPower);
    mElevatorFollower.set(pivotPower);
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
    return mPivotLeaderPID.atSetpoint();
  }

  /** Stop the arm motors and hold current position by updating the target. */
  public void stopArm() {
    mElevatorLeader.set(0);
    mCurrentTarget = mPivotLeaderEncoder.getPosition(); // Hold current spot
  }

  /** Increase the internal KP tuning value by the configured increment. */
  

  // Commands
  public Command IntakeToPositionCommand(double position) {
    return run(
        () -> {
          setTargetArmPosition(position);
        });
  }

  public Command runIntakePivotGroundCommand() {
    return run(
        () -> {
          if (getEncoderValue() < 0.7) {
            setTargetArmPosition(ElevatorConstants.PIVOT_OUT);
          } else {
            setArmPower(0);
          }
        });
  }

  public Command runIntakePivotBumpCommand() {
    return run(
        () -> {
          setTargetArmPosition(ElevatorConstants.PIVOT_BUMP);
        });
  }

  public Command runIntakePivotInCommand() {
    return run(
        () -> {
          setTargetArmPosition(ElevatorConstants.PIVOT_IN);
        });
  }

  public Command IntakeSlightlyUpCommand() {
    return run(
        () -> {
          setTargetArmPosition(ElevatorConstants.PIVOT_AGITATE);
        });
  }

  public Command stopPivotCommand() {
    return run(
        () -> {
          setArmPower(0.0);
        });
  }

  public Command runPivotCommand() {
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
    SmartDashboard.putNumber("Arm/Pivot Encoder Value", getEncoderValue());
    SmartDashboard.putNumber("Arm/Target Position", mCurrentTarget);
  }
}
