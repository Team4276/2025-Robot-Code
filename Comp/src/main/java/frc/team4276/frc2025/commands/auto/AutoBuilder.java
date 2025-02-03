package frc.team4276.frc2025.commands.auto;

import static frc.team4276.frc2025.commands.auto.AutoCommands.*;
import static frc.team4276.util.path.ChoreoUtil.*;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.frc2025.RobotState;
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

  public Command FiveCoral(){
    PathPlannerTrajectory traj1 = getPathPlannerTrajectoryFromChoreo("5Coral", 0);
    PathPlannerTrajectory traj2 = getPathPlannerTrajectoryFromChoreo("5Coral", 1);
    PathPlannerTrajectory traj3 = getPathPlannerTrajectoryFromChoreo("5Coral", 2);
    PathPlannerTrajectory traj4 = getPathPlannerTrajectoryFromChoreo("5Coral", 3);
    PathPlannerTrajectory traj5 = getPathPlannerTrajectoryFromChoreo("5Coral", 4);
    PathPlannerTrajectory traj6 = getPathPlannerTrajectoryFromChoreo("5Coral", 5);
    PathPlannerTrajectory traj7 = getPathPlannerTrajectoryFromChoreo("5Coral", 6);
    PathPlannerTrajectory traj8 = getPathPlannerTrajectoryFromChoreo("5Coral", 7);
    PathPlannerTrajectory traj9 = getPathPlannerTrajectoryFromChoreo("5Coral", 8);

    return Commands.sequence( // TODO: actually learn command based and fix this lol
        resetPose(traj1.getInitialPose()),

        driveWithSuperstructureCommand(drive, superstructure, traj1, Superstructure.Goal.L2),

        scoreCommand(superstructure),

        driveAndIntakeCommand(drive, superstructure, traj2),

        driveWithSuperstructureCommand(drive, superstructure, traj3, Superstructure.Goal.L2),

        scoreCommand(superstructure),

        driveAndIntakeCommand(drive, superstructure, traj4),

        driveWithSuperstructureCommand(drive, superstructure, traj5, Superstructure.Goal.L1),

        scoreCommand(superstructure),

        driveAndIntakeCommand(drive, superstructure, traj6),

        driveWithSuperstructureCommand(drive, superstructure, traj7, Superstructure.Goal.L2),

        scoreCommand(superstructure),

        driveAndIntakeCommand(drive, superstructure, traj8),

        driveWithSuperstructureCommand(drive, superstructure, traj9, Superstructure.Goal.L1),

        scoreCommand(superstructure)
      );
  }
}
