package frc.team4276.frc2025.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.frc2025.Constants;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.ScoringHelper;
import frc.team4276.frc2025.field.FieldConstants.Reef;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure;
import frc.team4276.util.dashboard.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class AutoScore {
  private static final LoggedTunableNumber reefAlignThreshold =
      new LoggedTunableNumber("AutoScore/ReefAlignThreshold", 1.0);
  private static final LoggedTunableNumber reefNudgeThreshold =
      new LoggedTunableNumber("AutoScore/ReefNudgeThreshold", 0.1);

  private static boolean cancelTxTy = false;

  public static Command coralScoreCommand(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Superstructure superstructure,
      ScoringHelper scoringHelper) {
    return Commands.sequence(
        Commands.waitUntil(
                () ->
                    scoringHelper
                            .getSelectedAlignPose()
                            .getTranslation()
                            .getDistance(
                                getRobotPose(
                                        scoringHelper.getSelectedReef(),
                                        scoringHelper.getSelectedAlignPose())
                                    .getTranslation())
                        < reefAlignThreshold.getAsDouble())
            .deadlineFor(
                DriveCommands.joystickDriveAtHeading(
                    drive,
                    xSupplier,
                    ySupplier,
                    () -> scoringHelper.getSelectedScorePose().getRotation().getRadians())),
        new DriveToPose(
                drive,
                scoringHelper::getSelectedAlignPose,
                () ->
                    getRobotPose(
                        scoringHelper.getSelectedReef(), scoringHelper.getSelectedAlignPose()))
            .until(
                () ->
                    (scoringHelper
                            .getSelectedAlignPose()
                            .getTranslation()
                            .getDistance(
                                getRobotPose(
                                        scoringHelper.getSelectedReef(),
                                        scoringHelper.getSelectedScorePose())
                                    .getTranslation())
                        < reefNudgeThreshold.getAsDouble())),
        new DriveToPose(
                drive,
                scoringHelper::getSelectedScorePose,
                () ->
                    getRobotPose(
                        scoringHelper.getSelectedReef(), scoringHelper.getSelectedScorePose()))
            .alongWith(superstructure.setGoalCommand(scoringHelper::getSuperstructureGoal)));
  }

  public static Pose2d getRobotPose(Reef reef, Pose2d finalPose) {
    return Constants.isSim || cancelTxTy
        ? RobotState.getInstance().getEstimatedPose()
        : RobotState.getInstance().getReefPose(reef.ordinal() / 2, finalPose);
  }

  public static Command bargeScoreCommand() {
    return Commands.none();
  }
}
