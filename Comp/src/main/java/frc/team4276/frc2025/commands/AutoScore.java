package frc.team4276.frc2025.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.ScoringHelper;
import frc.team4276.frc2025.field.FieldConstants.Reef;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure.Goal;
import frc.team4276.frc2025.subsystems.vision.Vision;
import frc.team4276.util.dashboard.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AutoScore {
  private static final LoggedTunableNumber reefAlignThreshold =
      new LoggedTunableNumber("AutoScore/ReefAlignThreshold", 1.0);
  private static final LoggedTunableNumber reefNudgeThreshold =
      new LoggedTunableNumber("AutoScore/ReefNudgeThreshold", 0.1);

  private static final LoggedTunableNumber sideAutoSelectTime =
      new LoggedTunableNumber("AutoScore/VisionAutoFaceSelectTime", 0.2);

  private static boolean cancelTxTy = false;

  private static Superstructure.Goal autoScoreLevel = Goal.L2;

  public static Command selectAutoScoreLevel(Superstructure.Goal goal) {
    return Commands.runOnce(() -> autoScoreLevel = goal);
  }

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
      boolean isLeft,
      Vision vision) {
    return coralAlignCommand(
        drive, xSupplier, ySupplier, superstructure, autoScoreLevel, () -> isLeft, vision);
  }

  public static Command coralAlignCommand(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Superstructure superstructure,
      Superstructure.Goal goal,
      BooleanSupplier isLeft,
      Vision vision) {
    Debouncer debouncer = new Debouncer(sideAutoSelectTime.getAsDouble());

    return Commands.waitUntil(() -> debouncer.calculate(getSide(vision) != -1))
        .andThen(
            coralAlignCommand(
                drive,
                xSupplier,
                ySupplier,
                superstructure,
                getReefFromSide(getSide(vision), isLeft),
                goal));
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
    Debouncer debouncer = new Debouncer(sideAutoSelectTime.getAsDouble());

    return Commands.waitUntil(() -> debouncer.calculate(getSide(vision) != -1))
        .andThen(
            coralLockCommand(
                drive,
                xSupplier,
                ySupplier,
                superstructure,
                getReefFromSide(getSide(vision), () -> true),
                goal));
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

  private static int getIdFromVision(Vision vision) {
    if (vision.getPriorityTargObs().size() == 0) {
      return -1;
    }

    int tag1 = vision.getPriorityTargObs().get(0).tagId();

    if (vision.getPriorityTargObs().size() == 1) {
      return tag1;
    }

    int tag2 = vision.getPriorityTargObs().get(1).tagId();

    if (tag1 == tag2) {
      return tag1;
    }

    Rotation2d rotation = RobotState.getInstance().getEstimatedPose().getRotation();

    if (rotation
            .minus(Reef.values()[getSideFromTagId(tag1) * 2].getAlign().getRotation())
            .getRadians()
        > rotation
            .minus(Reef.values()[getSideFromTagId(tag1) * 2].getAlign().getRotation())
            .getRadians()) {
      return tag2;
    } else {
      return tag1;
    }
  }

  private static int getSideFromTagId(int id) {
    return switch (id) {
      case 6 -> 5;
      case 7 -> 0;
      case 8 -> 1;
      case 9 -> 2;
      case 10 -> 3;
      case 11 -> 4;

      case 17 -> 1;
      case 18 -> 0;
      case 19 -> 5;
      case 20 -> 4;
      case 21 -> 3;
      case 22 -> 2;

      default -> -1;
    };
  }

  private static int getSide(Vision vision) {
    return getSideFromTagId(getIdFromVision(vision));
  }

  private static Reef getReefFromSide(int side, BooleanSupplier isLeft) {
    if (side > 1 && side < 5) {
      return Reef.values()[(side * 2) + (isLeft.getAsBoolean() ? 1 : 0)];

    } else if (side > 0) {
      return Reef.values()[(side * 2) + (isLeft.getAsBoolean() ? 0 : 1)];

    } else {
      return Reef.A;
    }
  }

  private static Pose2d getRobotPose(Reef reef, Pose2d finalPose) {
    return cancelTxTy
        ? RobotState.getInstance().getEstimatedPose()
        : RobotState.getInstance().getReefPose(reef.ordinal() / 2, finalPose);
  }
}
