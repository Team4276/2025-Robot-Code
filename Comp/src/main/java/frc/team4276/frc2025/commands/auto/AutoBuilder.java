package frc.team4276.frc2025.commands.auto;

import static frc.team4276.frc2025.commands.auto.AutoCommands.*;
import static frc.team4276.util.path.PPUtil.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team4276.frc2025.AutoSelector;
import frc.team4276.frc2025.AutoSelector.AutoQuestionResponse;
import frc.team4276.frc2025.Constants;
import frc.team4276.frc2025.Constants.Mode;
import frc.team4276.frc2025.commands.DriveTrajectory;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.drive.Drive.DriveMode;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure.Goal;
import java.util.ArrayList;
import java.util.List;

public class AutoBuilder {
  private final Drive drive; // TODO: add drive to pose auto
  private final Superstructure superstructure;
  private final AutoSelector autoSelector;

  public AutoBuilder(Drive drive, Superstructure superstructure, AutoSelector autoSelector) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.autoSelector = autoSelector;
  }

  public Command testTraj(String name) {
    var traj = getPathPlannerTrajectoryFromChoreo(name);

    return resetPose(traj.getInitialPose()).andThen(new DriveTrajectory(drive, traj));
  }

  public Command taxiAuto(String name) {
    // Check if need to flip paths to barge side
    boolean mirrorLengthwise = autoSelector.getResponses().get(0) == AutoQuestionResponse.NO;

    var traj = getPathPlannerTrajectoryFromChoreo(name, mirrorLengthwise);

    return Commands.sequence(
        resetPose(traj.getInitialPose()),
        Commands.waitSeconds(autoSelector.getDelayInput()),
        new DriveTrajectory(drive, traj));
  }

  public Command shrimpleOcrAuto(
      List<AutoQuestionResponse> reefs, List<AutoQuestionResponse> levels) {

    if (reefs.isEmpty() || levels.isEmpty()) {
      CommandScheduler.getInstance()
          .schedule(notificationCommand("Error invalid Coral Auto List is Empty"));

      return Commands.none();
    }

    // Check if need to flip paths to barge side
    boolean mirrorLengthwise = autoSelector.getResponses().get(0) == AutoQuestionResponse.NO;

    var scoringCommand = new SequentialCommandGroup();

    var traj1 =
        getPathPlannerTrajectoryFromChoreo("c_st_sc_" + reefs.get(0).toString(), mirrorLengthwise);

    for (int i = 0; i < reefs.size(); i++) {
      var scTraj =
          i == 0
              ? traj1
              : getPathPlannerTrajectoryFromChoreo(
                  "c_sc_FAR_" + reefs.get(i).toString(), mirrorLengthwise, 0);
      var intTraj =
          getPathPlannerTrajectoryFromChoreo(
              "c_sc_FAR_" + reefs.get(i).toString(), mirrorLengthwise, 1);

      scoringCommand.addCommands(
          Commands.parallel(
              new DriveTrajectory(drive, scTraj),
              Commands.waitSeconds(scTraj.getTotalTimeSeconds() - 0.75)
                  .andThen(
                      superstructure
                          .setGoalCommand(toGoal(levels.get(i)))
                          .withDeadline(
                              Commands.waitUntil(
                                      () ->
                                          drive.getMode() != DriveMode.TRAJECTORY
                                              && superstructure.atGoal())
                                  .andThen(scoreCommand(superstructure))))),
          superstructure
              .setGoalCommand(Goal.INTAKE)
              .withDeadline(
                  new DriveTrajectory(drive, intTraj)
                      .andThen(
                          Constants.getMode() == Mode.SIM
                              ? Commands.waitSeconds(intakeWaitTime)
                              : Commands.waitUntil(() -> superstructure.hasCoral()))));
    }

    return Commands.sequence(
        resetPose(traj1.getInitialPose()),
        Commands.waitSeconds(autoSelector.getDelayInput()),
        Commands.runOnce(() -> superstructure.overrideCoral(true)),
        scoringCommand);
  }

  public Command shrimpleOcrAuto(List<AutoQuestionResponse> reefs) {
    return shrimpleOcrAuto(reefs, reefsToLevels(reefs));
  }

  public Command rpShrimpleOcrAuto() {
    return shrimpleOcrAuto(List.of(AutoQuestionResponse.G))
        .withDeadline(
            Commands.waitUntil(
                () -> !superstructure.hasCoral() && superstructure.getGoal() == Goal.INTAKE));
  }

  public Command FEBAshrimpleOcrAuto() {
    return shrimpleOcrAuto(
        List.of(
            AutoQuestionResponse.F,
            AutoQuestionResponse.E,
            AutoQuestionResponse.B,
            AutoQuestionResponse.A),
        List.of(
            AutoQuestionResponse.L2,
            AutoQuestionResponse.L2,
            AutoQuestionResponse.L2,
            AutoQuestionResponse.L2));
  }

  public Command shrimpleOcrAuto() {
    List<AutoQuestionResponse> reefs = new ArrayList<>();

    for (int i = 1; i < autoSelector.getResponses().size(); i++) {
      int ordinal = autoSelector.getResponses().get(i).ordinal() - AutoQuestionResponse.A.ordinal();
      if (ordinal >= 0 && ordinal < 12) {
        reefs.add(autoSelector.getResponses().get(i));
      }
    }

    return shrimpleOcrAuto(reefs, reefsToLevels(reefs));
  }

  private Superstructure.Goal toGoal(AutoQuestionResponse response) {
    switch (response) {
      case L1_LEFT:
        return Superstructure.Goal.L1;

      case L1_RIGHT:
        return Superstructure.Goal.L1;

      case L2:
        return Superstructure.Goal.L2;

      case L3:
        return Superstructure.Goal.L3;

      default:
        return Superstructure.Goal.STOW;
    }
  }

  /** fms convention (A-L) */
  private final int[] scoringAvailable = {2, 2, 1, 1, 2, 2, 1, 1, 2, 2, 1, 1};

  public List<AutoQuestionResponse> reefsToLevels(List<AutoQuestionResponse> reefs) {
    if (reefs.isEmpty()) {
      return List.of();
    }

    List<AutoQuestionResponse> levels = new ArrayList<>();

    int[] totalAvailable = scoringAvailable;

    for (var reef : reefs) {
      int currentReef = reef.ordinal() - AutoQuestionResponse.A.ordinal();
      int available = totalAvailable[currentReef];

      if (available > 1) {
        levels.add(AutoQuestionResponse.L2);
        totalAvailable[currentReef]--;

      } else {
        levels.add(
            available % 2 == 0 ? AutoQuestionResponse.L1_LEFT : AutoQuestionResponse.L1_RIGHT);
        totalAvailable[currentReef]--;
      }
    }

    return levels;
  }
}
