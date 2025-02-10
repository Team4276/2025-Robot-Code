package frc.team4276.frc2025.commands.auto;

import static frc.team4276.frc2025.commands.auto.AutoCommands.*;
import static frc.team4276.util.path.ChoreoUtil.*;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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

  private Command cycle(PathPlannerTrajectory toIntake, PathPlannerTrajectory toScore, Goal goal,
      double delay) {
    return driveAndIntakeCommand(drive, superstructure, toIntake)
        .andThen(driveAndScoreCommand(drive, superstructure, toScore, goal, delay));
  }

  private Command cycleL1(PathPlannerTrajectory toIntake, PathPlannerTrajectory toScore, boolean isLeftL1,
      double delay) {
    return driveAndIntakeCommand(drive, superstructure, toIntake)
        .andThen(driveAndScoreL1Command(drive, superstructure, toScore, delay, isLeftL1));
  }

  /**
   * dashboard custom coral auto
   * 
   * @param isProcessorSide
   * @param start
   * @param station
   * @param reef
   * @return
   */
  public Command coralScoreAuto(
      int isProcessorSide,
      int start,
      int station,
      int reef) {

    String pathStart = autoSelector.getResponses().get(start) + "_start_" + autoSelector.getResponses().get(station)
        + "_station";
    String pathBody = autoSelector.getResponses().get(station) + "_station_" + autoSelector.getResponses().get(reef)
        + "_reef";

    boolean mirrorLengthwise = autoSelector.getResponses().get(isProcessorSide) == AutoQuestionResponse.NO;

    PathPlannerTrajectory[] trajs = {
        getPathPlannerTrajectoryFromChoreo(pathStart, mirrorLengthwise, 0),
        getPathPlannerTrajectoryFromChoreo(pathStart, mirrorLengthwise, 1),
        getPathPlannerTrajectoryFromChoreo(pathBody, mirrorLengthwise, 0),
        getPathPlannerTrajectoryFromChoreo(pathBody, mirrorLengthwise, 1),
        getPathPlannerTrajectoryFromChoreo(pathBody, mirrorLengthwise, 2),
        getPathPlannerTrajectoryFromChoreo(pathBody, mirrorLengthwise, 3),
        getPathPlannerTrajectoryFromChoreo(pathBody, mirrorLengthwise, 4),
        getPathPlannerTrajectoryFromChoreo(pathBody, mirrorLengthwise, 5),
        getPathPlannerTrajectoryFromChoreo(pathBody, mirrorLengthwise, 6)
    };

    return Commands.sequence(
        notificationCommand("Run path with "
            + (autoSelector.getResponses().get(isProcessorSide) == AutoQuestionResponse.YES ? "processor side"
                : "barge side")),
        resetPose(trajs[0].getInitialPose()),
        Commands.waitSeconds(autoSelector.getDelayInput()),
        driveAndScoreCommand(drive, superstructure, trajs[0], Goal.L2, 0.25),
        cycle(trajs[1], trajs[2], Goal.L2, 0.5).onlyIf(() -> autoSelector.getCoralInput() >= 2),
        cycleL1(trajs[3], trajs[4], true, 0.5).onlyIf(() -> autoSelector.getCoralInput() >= 3),
        cycle(trajs[5], trajs[6], Goal.L2, 0.5).onlyIf(() -> autoSelector.getCoralInput() >= 4),
        cycleL1(trajs[7], trajs[8], false, 0.5).onlyIf(() -> autoSelector.getCoralInput() >= 5));
  }

  /**
   * pre made coral auto
   * 
   * @param start
   * @param station
   * @param reef
   * @param coral
   * @return
   */
  public Command coralScoreAuto(
      AutoQuestionResponse start,
      AutoQuestionResponse station,
      AutoQuestionResponse reef,
      int coral) {

    String pathStart = start.toString() + "_start_" + station.toString() + "_station";
    String pathBody = station.toString() + "_station_" + reef.toString() + "_reef";

    boolean mirrorLengthwise = autoSelector.getResponses().get(0) == AutoQuestionResponse.NO;

    PathPlannerTrajectory[] trajs = {
        getPathPlannerTrajectoryFromChoreo(pathStart, mirrorLengthwise, 0),
        getPathPlannerTrajectoryFromChoreo(pathStart, mirrorLengthwise, 1),
        getPathPlannerTrajectoryFromChoreo(pathBody, mirrorLengthwise, 0),
        getPathPlannerTrajectoryFromChoreo(pathBody, mirrorLengthwise, 1),
        getPathPlannerTrajectoryFromChoreo(pathBody, mirrorLengthwise, 2),
        getPathPlannerTrajectoryFromChoreo(pathBody, mirrorLengthwise, 3),
        getPathPlannerTrajectoryFromChoreo(pathBody, mirrorLengthwise, 4),
        getPathPlannerTrajectoryFromChoreo(pathBody, mirrorLengthwise, 5),
        getPathPlannerTrajectoryFromChoreo(pathBody, mirrorLengthwise, 6)
    };

    return Commands.sequence(
        notificationCommand("Run path with "
            + (autoSelector.getResponses().get(0) == AutoQuestionResponse.YES ? "processor side"
                : "barge side")),
        resetPose(trajs[0].getInitialPose()),
        Commands.waitSeconds(autoSelector.getDelayInput()),
        driveAndScoreCommand(drive, superstructure, trajs[0], Goal.L1, 0.25),
        cycle(trajs[1], trajs[2], Goal.L2, 0.5).onlyIf(() -> coral >= 2),
        cycleL1(trajs[3], trajs[4], true, 0.5).onlyIf(() -> coral >= 3),
        cycle(trajs[5], trajs[6], Goal.L2, 0.5).onlyIf(() -> coral >= 4),
        cycleL1(trajs[7], trajs[8], false, 0.5).onlyIf(() -> coral >= 5));
  }

  public Command inner5Piece() {
    return coralScoreAuto(AutoQuestionResponse.FAR, AutoQuestionResponse.CLOSE,
        AutoQuestionResponse.MIDDLE, 5);
  }

  public Command outter5Piece() {
    return coralScoreAuto(AutoQuestionResponse.MIDDLE, AutoQuestionResponse.FAR,
        AutoQuestionResponse.FAR, 5);
  }

  /**
   * @param reefs
   * @param levels
   * @param stations
   * @param coral
   * @return
   */
  public Command neoCoralScoreAuto(
      List<AutoQuestionResponse> reefs,
      List<AutoQuestionResponse> levels,
      List<AutoQuestionResponse> stations) {

    if (reefs.isEmpty() || levels.isEmpty() || stations.isEmpty()) { // TODO: handle this well
      CommandScheduler.getInstance().schedule(notificationCommand("Error invalid Coral Auto List is Empty"));

      return Commands.none();
    }

    // Check if need to flip paths to barge side
    boolean mirrorLengthwise = autoSelector.getResponses().get(0) == AutoQuestionResponse.NO;

    // Get paths
    String pathStart = "c_st_" + reefs.get(0).toString() + "_" + stations.get(0);

    List<PathPlannerTrajectory> trajs = new ArrayList<>();

    trajs.add(getPathPlannerTrajectoryFromChoreo(pathStart, mirrorLengthwise, 0));
    trajs.add(getPathPlannerTrajectoryFromChoreo(pathStart, mirrorLengthwise, 1));

    for (int i = 1; i < reefs.size(); i++) {
      trajs.add(getPathPlannerTrajectoryFromChoreo("c_sc_" + stations.get(i) + "_" + reefs.get(i).toString(),
          mirrorLengthwise, 0));
      trajs.add(getPathPlannerTrajectoryFromChoreo("c_sc_" + stations.get(i) + "_" + reefs.get(i).toString(),
          mirrorLengthwise, 1));

    }

    Iterator iterator = new Iterator();

    return Commands.sequence(
        notificationCommand("Run path with "
            + (autoSelector.getResponses().get(0) == AutoQuestionResponse.YES ? "processor side"
                : "barge side")),
        resetPose(trajs.get(0).getInitialPose()),
        Commands.waitSeconds(autoSelector.getDelayInput()),
        // driveScoreIntake(trajs.get(0), trajs.get(1), toGoal(levels.get(0))),
        // driveScoreIntake(trajs.get(2), trajs.get(3), toGoal(levels.get(1))),
        // driveScoreIntake(trajs.get(4), trajs.get(5), toGoal(levels.get(2))),
        // driveScoreIntake(trajs.get(6), trajs.get(7), toGoal(levels.get(3))),
        // driveScoreIntake(trajs.get(8), trajs.get(9), toGoal(levels.get(4))));
        // TODO: try to fix this broken crap
        Commands.waitUntil(() -> (iterator.getWithoutIncrement() >= trajs.size() && superstructure.getGoal() == Goal.INTAKE && drive.isTrajectoryCompleted())).deadlineFor( 
            Commands.sequence(
                followTrajectory(drive, () -> trajs.get(iterator.get())),
                // TODO: add proximity checker for trajectories with superstructure stuff
                // or make it an approach auto
                superstructure.setGoalCommand(() -> toGoal(levels.get((iterator.getWithoutIncrement() - 1) / 2)))
                    .raceWith(Commands.waitUntil(superstructure::atGoal).andThen(scoreCommand(superstructure))),
                superstructure.setGoalCommand(Goal.INTAKE)
                    .withDeadline(followTrajectory(drive, () -> trajs.get(iterator.get()))
                        .andThen(Commands.waitSeconds(intakeWaitTime))))
                .repeatedly()));
  }

  private Command driveScoreIntake(PathPlannerTrajectory traj1, PathPlannerTrajectory traj2, Goal goal) {
    return Commands.sequence(
        followTrajectory(drive, traj1),
        superstructure.setGoalCommand(goal)
            .raceWith(Commands.waitUntil(superstructure::atGoal).andThen(scoreCommand(superstructure))),
        superstructure.setGoalCommand(Goal.INTAKE)
            .withDeadline(followTrajectory(drive, traj2)
                .andThen(Commands.waitSeconds(intakeWaitTime))));
  }

  public Command testNeo() {
    return neoCoralScoreAuto(
        List.of(
            AutoQuestionResponse.E,
            AutoQuestionResponse.A,
            AutoQuestionResponse.B,
            AutoQuestionResponse.E,
            AutoQuestionResponse.F),
        List.of(
            AutoQuestionResponse.L1_LEFT,
            AutoQuestionResponse.L2,
            AutoQuestionResponse.L2,
            AutoQuestionResponse.L2,
            AutoQuestionResponse.L2),
        List.of(
            AutoQuestionResponse.FAR,
            AutoQuestionResponse.FAR,
            AutoQuestionResponse.FAR,
            AutoQuestionResponse.FAR,
            AutoQuestionResponse.FAR));
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

  public class Iterator {
    private int i;

    public Iterator() {
      i = 0;
    }

    public Iterator(int startIndex) {
      i = startIndex;
    }

    public int get() {
      i++;
      return i - 1;
    }

    public int getWithoutIncrement() {
      return i;
    }

  }

  public boolean isValidNeoCoralScoreAuto(
      List<AutoQuestionResponse> reefs,
      List<AutoQuestionResponse> levels,
      List<AutoQuestionResponse> stations,
      int coral,
      boolean strict) {
    // TODO: impl

    return false;
  }
}
