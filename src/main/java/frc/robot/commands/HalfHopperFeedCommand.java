package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.roller.Roller;

/** Repeatedly bumps the Elevator to walk balls up the hopper while the intake feeds them in. */
public class HalfHopperFeedCommand extends SequentialCommandGroup {

  public HalfHopperFeedCommand(Elevator Elevator, Roller intake) {
    addCommands(
        new SequentialCommandGroup(
                Elevator.runIntakeElevatorBumpCommand().withTimeout(0.3),
                new WaitCommand(0.1),
                Elevator.runIntakeElevatorGroundCommand().withTimeout(0.3),
                new WaitCommand(0.1))
            .deadlineFor(intake.runIntakeForwardCommand())
            .repeatedly());
  }
}
