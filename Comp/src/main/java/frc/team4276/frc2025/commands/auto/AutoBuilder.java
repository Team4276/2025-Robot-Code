package frc.team4276.frc2025.commands.auto;

import static frc.team4276.frc2025.commands.auto.AutoCommands.*;
import static frc.team4276.util.path.ChoreoUtil.*;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team4276.frc2025.AutoSelector;
import frc.team4276.frc2025.AutoSelector.AutoQuestionResponse;
import frc.team4276.frc2025.subsystems.arm.Arm;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.roller.Roller;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure.Goal;
import frc.team4276.frc2025.subsystems.vision.Vision;
import java.util.ArrayList;
import java.util.List;

public class AutoBuilder { // TODO: fix auto not intaking
  private final Drive drive; // TODO: add drive to pose auto
  private final Superstructure superstructure;
  private final Arm arm;
  private final Roller roller;
  private final Vision vision;
  private final AutoSelector autoSelector;

  public AutoBuilder(
      Drive drive,
      Superstructure superstructure,
      Arm arm,
      Roller roller,
      Vision vision,
      AutoSelector autoSelector) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.arm = arm;
    this.roller = roller;
    this.vision = vision;
    this.autoSelector = autoSelector;
  }

  public Command testTraj(String name) {
    var traj = getPathPlannerTrajectoryFromChoreo(name);

    return resetPose(traj.getInitialPose()).andThen(followTrajectory(drive, traj));
  }

  public Command taxiAuto(String name) {
    // Check if need to flip paths to barge side
    boolean mirrorLengthwise = autoSelector.getResponses().get(0) == AutoQuestionResponse.NO;

    var traj = getPathPlannerTrajectoryFromChoreo(name, mirrorLengthwise);

    return Commands.sequence(
        resetPose(traj.getInitialPose()),
        Commands.waitSeconds(autoSelector.getDelayInput()),
        followTrajectory(drive, traj));
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
      scoringCommand.addCommands(
          vision.setCamerasEnabledCommand(true, true),
          followTrajectory(
              drive,
              i == 0
                  ? traj1
                  : getPathPlannerTrajectoryFromChoreo(
                      "c_sc_FAR_" + reefs.get(i).toString(), mirrorLengthwise, 0)),
          superstructure
              .setGoalCommand(toGoal(levels.get(i)))
              .withDeadline(
                  Commands.waitUntil(() -> superstructure.atGoal())
                      .andThen(scoreCommand(superstructure))),
          vision.setCamerasEnabledCommand(true, true),
          followTrajectory(
              drive,
              getPathPlannerTrajectoryFromChoreo(
                  "c_sc_FAR_" + reefs.get(i).toString(), mirrorLengthwise, 1)),
          superstructure.setGoalCommand(Goal.INTAKE).withTimeout(intakeWaitTime));
    }

    return Commands.sequence(
        resetPose(traj1.getInitialPose()),
        Commands.waitSeconds(autoSelector.getDelayInput()),
        scoringCommand);
  }

  public Command shrimpleOcrAuto(List<AutoQuestionResponse> reefs) {
    return shrimpleOcrAuto(reefs, reefsToLevels(reefs));
  }

  public Command rpShrimpleOcrAuto() {
    return shrimpleOcrAuto(List.of(AutoQuestionResponse.G))
        .withDeadline(
            Commands.waitUntil(() -> drive.getMode() == Drive.DriveMode.TRAJECTORY)
                .andThen(Commands.waitUntil(() -> drive.isTrajectoryCompleted()))
                .andThen(Commands.waitUntil(() -> drive.getMode() == Drive.DriveMode.TRAJECTORY)));
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

  public Command ECDshrimpleCoralAuto() {
    // Check if need to flip paths to barge side
    boolean mirrorLengthwise = autoSelector.getResponses().get(0) == AutoQuestionResponse.NO;

    PathPlannerTrajectory[] trajs = new PathPlannerTrajectory[9];
    for (int i = 0; i < 9; i++) {
      trajs[i] = getPathPlannerTrajectoryFromChoreo("c_f_ECD", mirrorLengthwise, i);
    }

    return Commands.sequence(
        resetPose(trajs[0].getInitialPose()),
        Commands.waitSeconds(autoSelector.getDelayInput()),
        vision.setCamerasEnabledCommand(true),
        followTrajectory(drive, trajs[0]),
        superstructure
            .setGoalCommand(Goal.L2)
            .withDeadline(
                Commands.waitUntil(() -> superstructure.atGoal())
                    .andThen(scoreCommand(superstructure))),
        vision.setCamerasEnabledCommand(false),
        followTrajectory(drive, trajs[1]),
        superstructure.setGoalCommand(Goal.INTAKE).withTimeout(intakeWaitTime),
        followTrajectory(drive, trajs[2]),
        superstructure
            .setGoalCommand(Goal.L1)
            .withDeadline(
                Commands.waitUntil(() -> superstructure.atGoal())
                    .andThen(scoreCommand(superstructure))),
        followTrajectory(drive, trajs[3]),
        superstructure.setGoalCommand(Goal.INTAKE).withTimeout(intakeWaitTime),
        followTrajectory(drive, trajs[4]),
        superstructure
            .setGoalCommand(Goal.L3)
            .withDeadline(
                Commands.waitUntil(() -> superstructure.atGoal())
                    .andThen(scoreCommand(superstructure))),
        followTrajectory(drive, trajs[5]),
        superstructure.setGoalCommand(Goal.INTAKE).withTimeout(intakeWaitTime),
        followTrajectory(drive, trajs[6]),
        superstructure
            .setGoalCommand(Goal.L2)
            .withDeadline(
                Commands.waitUntil(() -> superstructure.atGoal())
                    .andThen(scoreCommand(superstructure))),
        followTrajectory(drive, trajs[7]),
        superstructure.setGoalCommand(Goal.INTAKE).withTimeout(intakeWaitTime),
        followTrajectory(drive, trajs[8]),
        superstructure
            .setGoalCommand(Goal.L3)
            .withDeadline(
                Commands.waitUntil(() -> superstructure.atGoal())
                    .andThen(scoreCommand(superstructure))));
  }

  public Command FEBAshrimpleCoralAuto() {
    // Check if need to flip paths to barge side
    boolean mirrorLengthwise = autoSelector.getResponses().get(0) == AutoQuestionResponse.NO;

    PathPlannerTrajectory[] trajs = new PathPlannerTrajectory[9];
    for (int i = 0; i < 7; i++) {
      trajs[i] = getPathPlannerTrajectoryFromChoreo("c_f_FEBA", mirrorLengthwise, i);
    }

    return Commands.sequence(
        resetPose(trajs[0].getInitialPose()),
        Commands.waitSeconds(autoSelector.getDelayInput()),
        followTrajectory(drive, trajs[0]),
        superstructure
            .setGoalCommand(Goal.L2)
            .withDeadline(
                Commands.waitUntil(() -> superstructure.atGoal())
                    .andThen(scoreCommand(superstructure))),
        followTrajectory(drive, trajs[1]),
        superstructure.setGoalCommand(Goal.INTAKE).withTimeout(intakeWaitTime),
        followTrajectory(drive, trajs[2]),
        superstructure
            .setGoalCommand(Goal.L2)
            .withDeadline(
                Commands.waitUntil(() -> superstructure.atGoal())
                    .andThen(scoreCommand(superstructure))),
        followTrajectory(drive, trajs[3]),
        superstructure.setGoalCommand(Goal.INTAKE).withTimeout(intakeWaitTime),
        followTrajectory(drive, trajs[4]),
        superstructure
            .setGoalCommand(Goal.L2)
            .withDeadline(
                Commands.waitUntil(() -> superstructure.atGoal())
                    .andThen(scoreCommand(superstructure))),
        followTrajectory(drive, trajs[5]),
        superstructure.setGoalCommand(Goal.INTAKE).withTimeout(intakeWaitTime),
        followTrajectory(drive, trajs[6]),
        superstructure
            .setGoalCommand(Goal.L2)
            .withDeadline(
                Commands.waitUntil(() -> superstructure.atGoal())
                    .andThen(scoreCommand(superstructure))));
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
      boolean cancelLastIntake,
      AutoQuestionResponse startStation) {

    if (reefs.isEmpty() || levels.isEmpty() || stations.isEmpty()) {
      CommandScheduler.getInstance()
          .schedule(notificationCommand("Error invalid Coral Auto List is Empty"));

      return Commands.none();
    }

    List<PathPlannerTrajectory> trajs = new ArrayList<>();
    SequentialCommandGroup scoringCommands = new SequentialCommandGroup();

    // Check if need to flip paths to barge side
    boolean mirrorLengthwise = autoSelector.getResponses().get(0) == AutoQuestionResponse.NO;

    // Get paths
    if (startStation != AutoQuestionResponse.EMPTY) {
      trajs.add(
          getPathPlannerTrajectoryFromChoreo(
              "c_sc_" + startStation + "_" + reefs.get(0).toString(), mirrorLengthwise, 0));
      trajs.add(
          getPathPlannerTrajectoryFromChoreo(
              "c_sc_" + stations.get(0) + "_" + reefs.get(0).toString(), mirrorLengthwise, 1));

    } else {
      trajs.add(
          getPathPlannerTrajectoryFromChoreo(
              "c_st_sc_" + reefs.get(0).toString(), mirrorLengthwise));
      if (!(reefs.size() == 1 && cancelLastIntake)) {
        trajs.add(
            getPathPlannerTrajectoryFromChoreo(
                "c_sc_" + stations.get(0) + "_" + reefs.get(0).toString(), mirrorLengthwise, 1));
      }
    }

    for (int i = 1; i < reefs.size(); i++) {
      trajs.add(
          getPathPlannerTrajectoryFromChoreo(
              "c_sc_" + stations.get(i - 1) + "_" + reefs.get(i).toString(), mirrorLengthwise, 0));

      trajs.add(
          getPathPlannerTrajectoryFromChoreo(
              "c_sc_" + stations.get(i) + "_" + reefs.get(i).toString(), mirrorLengthwise, 1));
    }

    for (int i = 0; i < reefs.size(); i++) {
      scoringCommands.addCommands(
          followTrajectory(drive, trajs.get(i * 2), () -> false)
              .withDeadline(
                  Commands.waitUntil(drive::isTrajectoryCompleted)
                      .andThen(
                          superstructure
                              .setGoalCommand(toGoal(levels.get(i)))
                              .withDeadline(
                                  Commands.waitUntil(superstructure::atGoal)
                                      .andThen(scoreCommand(superstructure))))));

      if (!(i == reefs.size() - 1 && cancelLastIntake)) {
        scoringCommands.addCommands(
            superstructure
                .setGoalCommand(Superstructure.Goal.INTAKE)
                .withDeadline(
                    followTrajectory(drive, trajs.get((i * 2) + 1))
                        .andThen(Commands.waitSeconds(intakeWaitTime))));
      }
    }

    if (startStation != AutoQuestionResponse.EMPTY) {
      return scoringCommands;
    }

    return Commands.sequence(
        resetPose(trajs.get(0).getInitialPose()),
        Commands.waitSeconds(autoSelector.getDelayInput()),
        scoringCommands);
  }

  public Command coralScoreAuto(
      List<AutoQuestionResponse> reefs,
      List<AutoQuestionResponse> levels,
      List<AutoQuestionResponse> stations,
      boolean cancelLastIntake) {
    return coralScoreAuto(reefs, levels, stations, cancelLastIntake, AutoQuestionResponse.EMPTY);
  }

  public Command algaeStart(AutoQuestionResponse endStation) {
    List<PathPlannerTrajectory> trajs = new ArrayList<>();

    trajs.add(getPathPlannerTrajectoryFromChoreo("a_st_RIGHT"));
    trajs.add(getPathPlannerTrajectoryFromChoreo("a_cr_" + endStation.toString()));

    return Commands.sequence(
        resetPose(trajs.get(0).getInitialPose()),
        Commands.waitSeconds(autoSelector.getDelayInput()),
        followTrajectory(drive, trajs.get(0))
            .alongWith(
                Commands.waitSeconds(1.0)
                    .andThen(
                        arm.setGoalCommand(Arm.Goal.INTAKE)
                            .alongWith(roller.setGoalCommand(Roller.Goal.INTAKE)))
                    .raceWith(Commands.waitSeconds(1.5))),
        arm.setGoalCommand(Arm.Goal.SCORE)
            .withDeadline(
                Commands.waitSeconds(0.5)
                    .andThen(roller.setGoalCommand(Roller.Goal.SCORE).withTimeout(0.5))),
        superstructure
            .setGoalCommand(Superstructure.Goal.INTAKE)
            .withDeadline(followTrajectory(drive, trajs.get(1)))
            .andThen(Commands.waitSeconds(intakeWaitTime)));
  }

  public Command rpAuto() {
    return coralScoreAuto(
        List.of(AutoQuestionResponse.G),
        List.of(AutoQuestionResponse.L1_LEFT),
        List.of(AutoQuestionResponse.FAR),
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
            AutoQuestionResponse.B,
            AutoQuestionResponse.A,
            AutoQuestionResponse.C,
            AutoQuestionResponse.D),
        List.of(
            AutoQuestionResponse.L2,
            AutoQuestionResponse.L2,
            AutoQuestionResponse.L2,
            AutoQuestionResponse.L1_RIGHT,
            AutoQuestionResponse.L1_LEFT),
        List.of(
            AutoQuestionResponse.FAR,
            AutoQuestionResponse.FAR,
            AutoQuestionResponse.FAR,
            AutoQuestionResponse.FAR,
            AutoQuestionResponse.FAR),
        false);
  }

  public Command test3Coral() {
    return coralScoreAuto(
        List.of(AutoQuestionResponse.E, AutoQuestionResponse.D, AutoQuestionResponse.C),
        List.of(
            AutoQuestionResponse.L2, AutoQuestionResponse.L1_RIGHT, AutoQuestionResponse.L1_LEFT),
        List.of(AutoQuestionResponse.FAR, AutoQuestionResponse.FAR, AutoQuestionResponse.FAR),
        false);
  }

  public Command vanHybridAuto() {
    var command =
        algaeStart(AutoQuestionResponse.CLOSE)
            .andThen(
                coralScoreAuto(
                    List.of(
                        AutoQuestionResponse.A,
                        AutoQuestionResponse.A,
                        AutoQuestionResponse.B,
                        AutoQuestionResponse.B),
                    List.of(
                        AutoQuestionResponse.L2,
                        AutoQuestionResponse.L1_LEFT,
                        AutoQuestionResponse.L2,
                        AutoQuestionResponse.L1_RIGHT),
                    List.of(
                        AutoQuestionResponse.CLOSE,
                        AutoQuestionResponse.CLOSE,
                        AutoQuestionResponse.CLOSE,
                        AutoQuestionResponse.CLOSE),
                    false,
                    AutoQuestionResponse.CLOSE));
    return command;
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

  public boolean isValidCoralScoreAuto(
      List<AutoQuestionResponse> reefs,
      List<AutoQuestionResponse> levels,
      List<AutoQuestionResponse> stations,
      int coral,
      boolean strict) {
    // TODO: impl

    return false;
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
      int available = totalAvailable[reef.ordinal() - AutoQuestionResponse.A.ordinal()];

      if (available > 1) {
        levels.add(AutoQuestionResponse.L2);
        totalAvailable[reef.ordinal() - AutoQuestionResponse.A.ordinal()]--;

      } else {
        levels.add(
            available % 2 == 0 ? AutoQuestionResponse.L1_LEFT : AutoQuestionResponse.L1_RIGHT);
        totalAvailable[reef.ordinal() - AutoQuestionResponse.A.ordinal()]--;
      }
    }

    return levels;
  }
}
