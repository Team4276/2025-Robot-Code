package frc.team4276.frc2025.commands.auto;

import static frc.team4276.frc2025.commands.auto.AutoCommands.*;
import static frc.team4276.util.path.ChoreoUtil.*;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.AutoSelector.AutoQuestionResponses;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure;

public class AutoBuilder {
  private final Drive drive;
  private final Superstructure superstructure;

  public AutoBuilder(Drive drive, Superstructure superstructure) {
    this.drive = drive;
    this.superstructure = superstructure;
  }

  public Command testTraj(String name) {
    var traj = getPathPlannerTrajectoryFromChoreo(name);

    return Commands.runOnce(() -> RobotState.getInstance().resetPose(traj.getInitialPose()))
        .andThen(followTrajectory(drive, traj));
  }

  private Command cycle(PathPlannerTrajectory toIntake, PathPlannerTrajectory toScore, Superstructure.Goal goal,
      double delay) {
    return driveAndIntakeCommand(drive, superstructure, toIntake)
        .andThen(driveAndScoreCommand(drive, superstructure, toScore, goal, delay));
  }

  public Command CoralScoreAuto(
      boolean isProcessorSide,
      AutoQuestionResponses start,
      AutoQuestionResponses station,
      AutoQuestionResponses reef,
      int coral,
      double delay) {

    String pathName = start + "_start_" + station + "_station_" + reef + "_reef";

    PathPlannerTrajectory traj1 = getPathPlannerTrajectoryFromChoreo(pathName, 0);
    PathPlannerTrajectory traj2 = getPathPlannerTrajectoryFromChoreo(pathName, 1);
    PathPlannerTrajectory traj3 = getPathPlannerTrajectoryFromChoreo(pathName, 2);
    PathPlannerTrajectory traj4 = getPathPlannerTrajectoryFromChoreo(pathName, 3);
    PathPlannerTrajectory traj5 = getPathPlannerTrajectoryFromChoreo(pathName, 4);
    PathPlannerTrajectory traj6 = getPathPlannerTrajectoryFromChoreo(pathName, 5);
    PathPlannerTrajectory traj7 = getPathPlannerTrajectoryFromChoreo(pathName, 6);
    PathPlannerTrajectory traj8 = getPathPlannerTrajectoryFromChoreo(pathName, 7);
    PathPlannerTrajectory traj9 = getPathPlannerTrajectoryFromChoreo(pathName, 8);

    return Commands.sequence(
        resetPose(traj1.getInitialPose()),
        driveAndScoreCommand(drive, superstructure, traj1, Superstructure.Goal.L2, 0.0),
        cycle(traj2, traj3, Superstructure.Goal.L2, 0.5).onlyIf(() -> coral >= 2),
        cycle(traj4, traj5, Superstructure.Goal.L1, 0.5).onlyIf(() -> coral >= 3),
        cycle(traj6, traj7, Superstructure.Goal.L2, 0.5).onlyIf(() -> coral >= 4),
        cycle(traj8, traj9, Superstructure.Goal.L1, 0.5).onlyIf(() -> coral >= 5));
  }
}
