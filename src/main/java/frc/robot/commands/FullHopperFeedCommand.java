package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.Roller;
import frc.robot.subsystems.shooter.Elevator;

/**
 * Agitates hard for a burst to clear a full hopper, then transitions into the lighter,
 * indefinitely-repeating {@link HalfHopperFeedCommand} behavior for the remainder of the hold.
 */
public class FullHopperFeedCommand extends SequentialCommandGroup {

  public FullHopperFeedCommand(Elevator pivot, Roller intake) {
    addCommands(
        pivot.IntakeSlightlyUpCommand().withTimeout(0.3),
        new WaitCommand(0.1),
        pivot.IntakeSlightlyUpCommand().withTimeout(0.3),
        new WaitCommand(0.1),
        pivot.IntakeSlightlyUpCommand().withTimeout(0.3),
        new WaitCommand(0.1),
        pivot.IntakeSlightlyUpCommand().withTimeout(0.3),
        new WaitCommand(0.1),
        pivot.IntakeSlightlyUpCommand().withTimeout(0.3),
        new WaitCommand(0.1),
        pivot.IntakeSlightlyUpCommand().withTimeout(0.3),
        new WaitCommand(0.1),
        new HalfHopperFeedCommand(pivot, intake).repeatedly());
  }
} // TODO: pretty sure .repeatedly doesnt work
