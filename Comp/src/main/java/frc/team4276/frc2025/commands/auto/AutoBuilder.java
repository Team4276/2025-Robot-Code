package frc.team4276.frc2025.commands.auto;

import static frc.team4276.frc2025.commands.auto.AutoCommands.*;
import static frc.team4276.util.path.ChoreoUtil.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
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

  public Command coralScoreAuto(
      Supplier<AutoQuestionResponse> isProcessorSide,
      Supplier<AutoQuestionResponse> start,
      Supplier<AutoQuestionResponse> station,
      Supplier<AutoQuestionResponse> reef,
      Supplier<Integer> coral,
      DoubleSupplier delay) {

    String pathStart = start.get().toString() + "_start_" + station.get().toString() + "_station";
    String pathBody = station.get().toString() + "_station_" + reef.get().toString() + "_reef";

    boolean mirrorLengthwise = isProcessorSide.get() == AutoQuestionResponse.NO;

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
        alertCommand("Run path with "
            + (isProcessorSide.get() == AutoQuestionResponse.YES ? "processor side" : "barge side")),
        resetPose(trajs[0].getInitialPose()),
        Commands.waitSeconds(delay.getAsDouble()),
        driveAndScoreCommand(drive, superstructure, trajs[0], Goal.L2, 0.25),
        cycle(trajs[1], trajs[2], Goal.L2, 0.5).onlyIf(() -> coral.get() >= 2),
        cycleL1(trajs[3], trajs[4], true, 0.5).onlyIf(() -> coral.get() >= 3),
        cycle(trajs[5], trajs[6], Goal.L2, 0.5).onlyIf(() -> coral.get() >= 4),
        cycleL1(trajs[7], trajs[8], false, 0.5).onlyIf(() -> coral.get() >= 5));
  }

  public Command coralScoreAuto(
      Supplier<AutoQuestionResponse> isProcessorSide,
      AutoQuestionResponse start,
      AutoQuestionResponse station,
      AutoQuestionResponse reef,
      int coral,
      double delay) {
    return coralScoreAuto(
        isProcessorSide,
        () -> start,
        () -> station,
        () -> reef,
        () -> coral,
        () -> delay);
  }

  public Command inner5Piece(Supplier<AutoQuestionResponse> isProcessorSide, DoubleSupplier delay) {
    return coralScoreAuto(isProcessorSide, () -> AutoQuestionResponse.FAR, () -> AutoQuestionResponse.CLOSE,
        () -> AutoQuestionResponse.MIDDLE, () -> 5, delay);
  }

  public Command outter5Piece(Supplier<AutoQuestionResponse> isProcessorSide, DoubleSupplier delay) {
    return coralScoreAuto(isProcessorSide, () -> AutoQuestionResponse.MIDDLE, () -> AutoQuestionResponse.FAR,
        () -> AutoQuestionResponse.FAR, () -> 5, delay);
  }
}
