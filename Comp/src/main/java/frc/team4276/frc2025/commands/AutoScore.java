package frc.team4276.frc2025.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.ScoringHelper;
import frc.team4276.frc2025.field.FieldConstants.Reef;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure;
import frc.team4276.frc2025.subsystems.vision.Vision;
import frc.team4276.util.dashboard.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AutoScore {
  private static final LoggedTunableNumber reefAlignThreshold =
      new LoggedTunableNumber("AutoScore/ReefAlignThreshold", 1.0);
  private static final LoggedTunableNumber reefNudgeThreshold =
      new LoggedTunableNumber("AutoScore/ReefNudgeThreshold", 0.1);

  private static boolean cancelTxTy = false;

  /** Append as a parrallel command group to coral align commands */
  public static Command autoScoreCommand(Superstructure superstructure) {
    return Commands.waitUntil(
            () ->
                superstructure.getGoal() != Superstructure.Goal.STOW
                    && superstructure.atGoal()
                    && DriveToPose.atGoal())
        .andThen(superstructure.scoreCommand(false));
  }

  public static Command coralAlignCommand(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Superstructure superstructure,
      Superstructure.Goal goal,
      BooleanSupplier isLeft,
      Vision vision) {
    // TODO: impl
    return coralAlignCommand(drive, xSupplier, ySupplier, superstructure, null, goal);
  }

  public static Command coralAlignCommand(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Superstructure superstructure,
      ScoringHelper scoringHelper) {
    return coralAlignCommand(
        drive,
        xSupplier,
        ySupplier,
        superstructure,
        scoringHelper.getSelectedReef(),
        scoringHelper.getSuperstructureGoal());
  }

  public static Command coralAlignCommand(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Superstructure superstructure,
      Reef reef,
      Superstructure.Goal goal) {
    return Commands.sequence(
        Commands.waitUntil(
                () ->
                    reef.getAlign()
                            .getTranslation()
                            .getDistance(getRobotPose(reef, reef.getAlign()).getTranslation())
                        < reefAlignThreshold.getAsDouble())
            .deadlineFor(
                DriveCommands.joystickDriveAtHeading(
                    drive, xSupplier, ySupplier, () -> reef.getScore().getRotation().getRadians())),
        new DriveToPose(drive, () -> reef.getAlign(), () -> getRobotPose(reef, reef.getAlign()))
            .until(
                () ->
                    (reef.getAlign()
                            .getTranslation()
                            .getDistance(getRobotPose(reef, reef.getScore()).getTranslation())
                        < reefNudgeThreshold.getAsDouble())),
        new DriveToPose(drive, () -> reef.getScore(), () -> getRobotPose(reef, reef.getScore()))
            .alongWith(superstructure.setGoalCommand(goal)));
  }

  public static Command coralLockCommand(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Superstructure superstructure,
      Superstructure.Goal goal,
      Vision vision) {

    // TODO: impl
    return coralLockCommand(drive, xSupplier, ySupplier, superstructure, null, goal);
  }

  public static Command coralLockCommand(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Superstructure superstructure,
      Reef reef,
      Superstructure.Goal goal) {
    return DriveCommands.joystickDriveAtHeading(
            drive, xSupplier, ySupplier, () -> reef.getScore().getRotation().getRadians())
        .alongWith(superstructure.setGoalCommand(goal));
  }

  public static Command bargeScoreCommand() {
    return Commands.none();
  }

  private static Pose2d getRobotPose(Reef reef, Pose2d finalPose) {
    return cancelTxTy
        ? RobotState.getInstance().getEstimatedPose()
        : RobotState.getInstance().getReefPose(reef.ordinal() / 2, finalPose);
  }
}
