

package frc.robot.subsystems.shooter;

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
import frc.robot.subsystems.intake.RollerConfig;
import frc.robot.subsystems.intake.RollerConstants;
import frc.robot.subsystems.intake.RollerConfig.ArmConfig;
import frc.robot.subsystems.intake.RollerConstants.ArmConstants;

public class Elevator extends SubsystemBase {
  private final SparkMax mPivotLeader;
  private final SparkMax mPivotFollower;
  private final AbsoluteEncoder mPivotFollowerEncoder;
  private final AbsoluteEncoder mPivotLeaderEncoder;
  private final PIDController mPivotLeaderPID;
  private final PIDController mPivotFollowerPID;

  public double mPivotOutputPower = 0;
  
  private double mCurrentTarget = RollerConstants.ArmConstants.PIVOT_IN;
  private boolean mIntakeGroundSecondStage = false;

  public Elevator() {
    mPivotLeader = new SparkMax(RollerConstants.ArmConstants.ARM_LEADER_ID, MotorType.kBrushless);
    mPivotFollower =
        new SparkMax(RollerConstants.ArmConstants.ARM_FOLLOWER_ID, MotorType.kBrushless);

    // Applying configuration
    mPivotLeader.configure(
        RollerConfig.ArmConfig.pivotLeaderConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    mPivotFollower.configure(
        RollerConfig.ArmConfig.pivotFollowerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    mPivotFollowerEncoder = mPivotFollower.getAbsoluteEncoder();
    mPivotLeaderEncoder = mPivotLeader.getAbsoluteEncoder();

    // PID constants for arm
    mPivotFollowerPID =
        new PIDController(
            RollerConstants.ArmConstants.ARM_KP,
            RollerConstants.ArmConstants.ARM_KI,
            RollerConstants.ArmConstants.ARM_KD);

    mPivotLeaderPID =
        new PIDController(
            RollerConstants.ArmConstants.ARM_KP,
            RollerConstants.ArmConstants.ARM_KI,
            RollerConstants.ArmConstants.ARM_KD);

    mPivotLeaderPID.setTolerance(RollerConstants.ArmConstants.POSITION_TOLERANCE);
    mPivotFollowerPID.setTolerance(RollerConstants.ArmConstants.POSITION_TOLERANCE);
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
        mPivotFollowerPID.calculate(mPivotFollowerEncoder.getPosition(), mCurrentTarget);
    mPivotFollower.set(mPivotOutputPower);
    mPivotLeader.set(mPivotOutputPower);
  }

  /**
   * Set arm pivot motor power directly (open-loop).
   *
   * @param pivotPower motor power in range [-1.0, 1.0]
   */
  public void setArmPower(double pivotPower) {
    mPivotLeader.set(pivotPower);
    mPivotFollower.set(pivotPower);
  }

  /**
   * Get the follower absolute encoder position.
   *
   * @return encoder position
   */
  public double getEncoderValue() {
    return mPivotFollowerEncoder.getPosition();
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
    mPivotLeader.set(0);
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
            setTargetArmPosition(RollerConstants.ArmConstants.PIVOT_OUT);
          } else {
            setArmPower(0);
          }
        });
  }

  public Command runIntakePivotBumpCommand() {
    return run(
        () -> {
          setTargetArmPosition(RollerConstants.ArmConstants.PIVOT_BUMP);
        });
  }

  public Command runIntakePivotInCommand() {
    return run(
        () -> {
          setTargetArmPosition(RollerConstants.ArmConstants.PIVOT_IN);
        });
  }

  public Command IntakeSlightlyUpCommand() {
    return run(
        () -> {
          setTargetArmPosition(RollerConstants.ArmConstants.PIVOT_AGITATE);
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
