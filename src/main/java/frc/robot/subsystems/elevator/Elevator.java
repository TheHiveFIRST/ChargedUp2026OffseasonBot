

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
  // current target position for the elevator 
  private double mCurrentTarget = ElevatorConstants.Elevator_IN_HUMAN;
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

    // PID constants for elevator control
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
    mElevatorFollower.set(0);
    mCurrentTarget = mElevatorLeaderEncoder.getPosition(); // Hold current spot
  }
  

  // Commands

  // Command to move the elevator to a target position
  public Command IntakeToPositionCommand(double position) {
    return run(
        () -> {
          setTargetArmPosition(position);
        });
  }

  // Command to move the elevator to ground intaking position, controlled stop when the elevator reaches a certain height
  public Command runIntakeElevatorGroundCommand() {
    return run(
        () -> {
          if (getEncoderValue() < 0.7) {
            setTargetArmPosition(ElevatorConstants.Elevator_IN_GROUND);
          } else {
            setArmPower(0);
          }
        });
  }

  // Command to move the elevator to human intaking position
  public Command runIntakeElevatorHumanCommand() {
    return run(
        () -> {
          setTargetArmPosition(ElevatorConstants.Elevator_IN_HUMAN);
        });
  }

  // Command to move the elevator to mid outtaking position
  public Command runOutakeElevatorMidCommand() {
    return run(
        () -> {
          setTargetArmPosition(ElevatorConstants.Elevator_OUT_MID);
        });
  }

  // Command to move the elevator to high outtaking position
  public Command runOutakeElevatorHighCommand() {
    return run(
        () -> {
          setTargetArmPosition(ElevatorConstants.Elevator_OUT_HIGH);
        });
  }

  public Command stopElevatorCommand() {
    return run(
        () -> {
          setArmPower(0.0);
        });
  }

  // Run elevator motors at set speed
  public Command runElevatorCommand() {
    return run(
        () -> {
          setArmPower(ElevatorConstants.Elevator_RUN_SPEED);
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
