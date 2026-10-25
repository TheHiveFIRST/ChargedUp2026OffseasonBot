package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GameObjectConstants;
import frc.robot.subsystems.intake.Roller;
import frc.robot.subsystems.shooter.Elevator;
import frc.robot.subsystems.shooter.Elevator;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ShootCommands {

  private ShootCommands() {}

  /** Continuously computes distance from the robot's current pose to the hub, in meters. */
  public static DoubleSupplier hubDistanceSupplier(Drive drive) {
    return () -> getShotDistance(GameObjectConstants.getHubPoseTranslation2D(), drive);
  }

  /** Continuously computes the field-relative bearing from the robot's current pose to the hub. */
  public static Supplier<Rotation2d> hubHeadingSupplier(Drive drive) {
    return () ->
        GameObjectConstants.getHubPoseTranslation2D()
            .minus(drive.getPose().getTranslation())
            .getAngle();
  }

  // gets the distance from the hub to the robot
  public static double getShotDistance(Translation2d targetPose, Drive drive) {
    Pose2d odometryPose = drive.getPose();
    double shooterToTargetMeters = odometryPose.getTranslation().getDistance(targetPose);
    return shooterToTargetMeters;
  }

  /**
   * Auto-aims the drivetrain at the hub while the driver retains translational control, and
   * continuously spins up + feeds the shooter (regression RPM updates live from odometry) along
   * with agitating the hopper to feed balls in. Intended for a whileTrue binding.
   */
  
}
