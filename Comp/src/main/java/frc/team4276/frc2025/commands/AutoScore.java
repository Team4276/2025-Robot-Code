package frc.team4276.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.ScoringHelper;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure;
import frc.team4276.frc2025.subsystems.vision.Vision;
import frc.team4276.util.dashboard.LoggedTunableNumber;

public class AutoScore {
  private static final LoggedTunableNumber reefAlignThreshold =
      new LoggedTunableNumber("AutoScore/ReefAlignThreshold", 2.0);
  private static final LoggedTunableNumber reefNudgeThreshold =
      new LoggedTunableNumber("AutoScore/ReefNudgeThreshold", 0.5);

  public static Command getAutoScoreCommand(
      Drive drive, Superstructure superstructure, Vision vision, ScoringHelper scoringHelper) {
    return Commands.sequence(
            Commands.waitUntil(
                    () ->
                        (scoringHelper
                                .getSelectedAlignPose()
                                .getTranslation()
                                .getDistance(
                                    RobotState.getInstance().getEstimatedPose().getTranslation())
                            < reefAlignThreshold.getAsDouble()))
                .deadlineFor(
                    DriveCommands.headingAlignCommand(
                        drive, () -> scoringHelper.getSelectedScorePose().getRotation())),
            DriveCommands.driveToPoseCommand(drive, scoringHelper::getSelectedAlignPose)
                .until(
                    () ->
                        (scoringHelper
                                    .getSelectedAlignPose()
                                    .getTranslation()
                                    .getDistance(
                                        RobotState.getInstance()
                                            .getEstimatedPose()
                                            .getTranslation())
                                < reefNudgeThreshold.getAsDouble()
                            && drive.isAutoHeadingAligned())),
            DriveCommands.driveToPoseCommand(drive, scoringHelper::getSelectedScorePose)
                .until(drive::isAutoAligned)
                .alongWith(superstructure.setGoalCommand(scoringHelper::getSuperstructureGoal)))
        .alongWith(vision.setEnableCameraCommand(1, false))
        .finallyDo(() -> vision.setEnableCamera(1, true));
  }

  public static Command getAutoHeadingAlignScoreCommand(
      Drive drive, Superstructure superstructure, ScoringHelper scoringHelper) {
    return DriveCommands.headingAlignCommand(
            drive, () -> scoringHelper.getSelectedScorePose().getRotation())
        .alongWith(superstructure.setGoalCommand(scoringHelper::getSuperstructureGoal));
  }
}
