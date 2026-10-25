package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.intake.Roller;
import frc.robot.subsystems.shooter.Elevator;

/** Extends the pivot out to the ground and runs the intake roller forward. */
public class IntakeDeployCommand extends ParallelCommandGroup {

  public IntakeDeployCommand(Elevator pivot, Roller intake) {
    addCommands(pivot.runIntakePivotGroundCommand(), intake.runIntakeForwardCommand());
  }
}
