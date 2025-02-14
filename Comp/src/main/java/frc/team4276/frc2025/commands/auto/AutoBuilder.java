package frc.team4276.frc2025.commands.auto;

import static frc.team4276.frc2025.commands.auto.AutoCommands.*;
import static frc.team4276.util.path.ChoreoUtil.*;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team4276.frc2025.AutoSelector;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.AutoSelector.AutoQuestionResponse;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure.Goal;

public class AutoBuilder {
  private final Drive drive;
  private final Superstructure superstructure;
  private final AutoSelector autoSelector;

  public AutoBuilder(Drive drive, Superstructure superstructure, AutoSelector autoSelector) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.autoSelector = autoSelector;
  }

  public Command testTraj(String name) {
    var traj = getPathPlannerTrajectoryFromChoreo(name);

    return Commands.runOnce(() -> RobotState.getInstance().resetPose(traj.getInitialPose()))
        .andThen(followTrajectory(drive, traj));
  }

  public Command taxiAuto(String name) {
    // Check if need to flip paths to barge side
    boolean mirrorLengthwise = autoSelector.getResponses().get(0) == AutoQuestionResponse.NO;

    var traj = getPathPlannerTrajectoryFromChoreo(name, mirrorLengthwise);

    return Commands.sequence(
        notificationCommand("Run path on " + (mirrorLengthwise ? "barge side" : "processor side")),
        resetPose(traj.getInitialPose()),
        Commands.waitSeconds(autoSelector.getDelayInput()),
        followTrajectory(drive, traj));
  }

  /**
   * @param reefs
   * @param levels
   * @param stations
   * @param coral
   * @return
   */
  public Command coralScoreAuto(
      List<AutoQuestionResponse> reefs,
      List<AutoQuestionResponse> levels,
      List<AutoQuestionResponse> stations,
      boolean cancelLastIntake) {

    if (reefs.isEmpty() || levels.isEmpty() || stations.isEmpty()) { // TODO: handle this well
      CommandScheduler.getInstance().schedule(notificationCommand("Error invalid Coral Auto List is Empty"));

      return Commands.none();
    }

    List<PathPlannerTrajectory> trajs = new ArrayList<>();
    SequentialCommandGroup scoringCommands = new SequentialCommandGroup();

    // Check if need to flip paths to barge side
    boolean mirrorLengthwise = autoSelector.getResponses().get(0) == AutoQuestionResponse.NO;

    // Get paths
    trajs.add(getPathPlannerTrajectoryFromChoreo("c_st_sc_" + reefs.get(0).toString(), mirrorLengthwise));
    trajs.add(getPathPlannerTrajectoryFromChoreo("c_sc_" + stations.get(0) + "_" + reefs.get(0).toString(), mirrorLengthwise, 1));

    for (int i = 1; i < reefs.size(); i++) {
      trajs.add(getPathPlannerTrajectoryFromChoreo("c_sc_" + stations.get(i) + "_" + reefs.get(i).toString(),
          mirrorLengthwise, 0));
      trajs.add(getPathPlannerTrajectoryFromChoreo("c_sc_" + stations.get(i) + "_" + reefs.get(i).toString(),
          mirrorLengthwise, 1));
    }

    for (int i = 0; i < reefs.size(); i++) {
      scoringCommands.addCommands(
        followTrajectory(drive, trajs.get(i * 2)),
        superstructure.setGoalCommand(toGoal(levels.get(i)))
            .raceWith(Commands.waitUntil(superstructure::atGoal).andThen(scoreCommand(superstructure))));

      if(i != reefs.size() - 1 && !cancelLastIntake){
        scoringCommands.addCommands(superstructure.setGoalCommand(Goal.INTAKE)
        .withDeadline(followTrajectory(drive, trajs.get((i * 2) + 1)))
        .andThen(Commands.waitSeconds(intakeWaitTime)));
      }
    }

    return Commands.sequence(
        notificationCommand("Run path on " + (mirrorLengthwise ? "barge side" : "processor side")),
        resetPose(trajs.get(0).getInitialPose()),
        Commands.waitSeconds(autoSelector.getDelayInput()),
        scoringCommands);
  }

  public Command coralScoreAuto(
      List<AutoQuestionResponse> reefs,
      List<AutoQuestionResponse> levels,
      List<AutoQuestionResponse> stations,
      int cancelLastIntake) {

    return coralScoreAuto(reefs, levels, stations,
        autoSelector.getResponses().get(cancelLastIntake) == AutoQuestionResponse.YES);
  }

  public Command rpAuto() {
    return coralScoreAuto(
        List.of(
            AutoQuestionResponse.G),
        List.of(
            AutoQuestionResponse.L1_LEFT),
        List.of(
            AutoQuestionResponse.FAR),
        true);
  }

  public Command max5Coral() {
    return coralScoreAuto(
        List.of(
            AutoQuestionResponse.F,
            AutoQuestionResponse.A,
            AutoQuestionResponse.B,
            AutoQuestionResponse.E,
            AutoQuestionResponse.E),
        List.of(
            AutoQuestionResponse.L2,
            AutoQuestionResponse.L2,
            AutoQuestionResponse.L2,
            AutoQuestionResponse.L2,
            AutoQuestionResponse.L1_LEFT),
        List.of(
            AutoQuestionResponse.FAR,
            AutoQuestionResponse.FAR,
            AutoQuestionResponse.FAR,
            AutoQuestionResponse.FAR,
            AutoQuestionResponse.FAR),
        false);
  }

  public Command safe5Coal() {
    return coralScoreAuto(
        List.of(
            AutoQuestionResponse.E,
            AutoQuestionResponse.D,
            AutoQuestionResponse.C,
            AutoQuestionResponse.B,
            AutoQuestionResponse.A),
        List.of(
            AutoQuestionResponse.L2,
            AutoQuestionResponse.L1_RIGHT,
            AutoQuestionResponse.L1_LEFT,
            AutoQuestionResponse.L2,
            AutoQuestionResponse.L2),
        List.of(
            AutoQuestionResponse.FAR,
            AutoQuestionResponse.FAR,
            AutoQuestionResponse.FAR,
            AutoQuestionResponse.FAR,
            AutoQuestionResponse.FAR),
        false);
  }

  private Goal toGoal(AutoQuestionResponse response) {
    switch (response) {
      case L1_LEFT:
        return Goal.L1;

      case L1_RIGHT:
        return Goal.L1;

      case L2:
        return Goal.L2;

      case L3:
        return Goal.L3;

      default:
        return Goal.STOW;
    }
  }

  public boolean isValidCoralScoreAuto(
      List<AutoQuestionResponse> reefs,
      List<AutoQuestionResponse> levels,
      List<AutoQuestionResponse> stations,
      int coral,
      boolean strict) {
    // TODO: impl

    return false;
  }
}
