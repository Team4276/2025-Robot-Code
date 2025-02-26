package frc.team4276.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.frc2025.Constants;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.ScoringHelper;
import frc.team4276.frc2025.Constants.RobotType;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure;
import frc.team4276.frc2025.subsystems.vision.Vision;
import frc.team4276.util.dashboard.LoggedTunableNumber;

public class AutoScore {
  private static final LoggedTunableNumber reefAlignThreshold = new LoggedTunableNumber(
      "AutoScore/ReefAlignThreshold",
      3.0);

  public static Command getAutoScoreCommand(
      Drive drive,
      Superstructure superstructure,
      Vision vision,
      ScoringHelper scoringHelper) {
    return Commands.sequence(
        Commands.waitUntil(() -> (scoringHelper.getSelectedAlignPose().getTranslation().getDistance(
            RobotState.getInstance().getEstimatedPose().getTranslation()) < reefAlignThreshold
                .getAsDouble())),
        DriveCommands.driveToPoseCommand(drive, scoringHelper::getSelectedAlignPose)
            .until(() -> drive.isAutoAligned()
                && (Constants.getType() == RobotType.SIMBOT ? true : superstructure.atGoal())),
        DriveCommands.driveToPoseCommand(drive, scoringHelper::getSelectedScorePose).until(drive::isAutoAligned)
            .alongWith(
                superstructure.setGoalCommand(scoringHelper::getSuperstructureGoal)))
        .alongWith(vision.setEnableCameraCommand(1, false))
        .finallyDo(() -> vision.setEnableCamera(1, false));
  }
}
