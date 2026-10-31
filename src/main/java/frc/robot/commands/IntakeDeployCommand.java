package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.roller.Roller;

/** Extends the Elevator out to the ground and runs the intake roller forward. */
public class IntakeDeployCommand extends ParallelCommandGroup {

  public IntakeDeployCommand(Elevator Elevator, Roller intake) {
    addCommands(Elevator.runIntakeElevatorGroundCommand(), intake.runIntakeForwardCommand());
  }
}
