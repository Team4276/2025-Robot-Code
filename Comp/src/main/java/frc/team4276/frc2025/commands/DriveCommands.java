package frc.team4276.frc2025.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.frc2025.subsystems.drive.Drive;

public class DriveCommands {

  public static Command driveToPoseCommand(Drive drive, Supplier<Pose2d> pose) {
    return Commands.startEnd(
        () -> drive
            .setAutoAlignPosition(pose.get()),
        drive::disableAutoAlign);
  }

  public static Command headingAlignCommand(Drive drive, Supplier<Rotation2d> rotation) {
    return Commands.startEnd(
        () -> drive.setHeadingGoal(rotation),
        drive::clearHeadingGoal);
  }
}
