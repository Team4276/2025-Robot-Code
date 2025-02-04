package frc.team4276.frc2025.commands.auto;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.field.FieldConstants;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure;
import frc.team4276.util.AllianceFlipUtil;

import java.util.function.Supplier;

public class AutoCommands {
  private AutoCommands() {
  }

  private static final double scoreWaitTime = 0.5;
  private static final double intakeWaitTime = 1.0;

  public static Command resetPose(Pose2d pose) {
    return Commands.runOnce(() -> RobotState.getInstance().resetPose(pose));
  }

  /**
   * Creates a command that follows a trajectory, command ends when the trajectory
   * is finished
   */
  public static Command followTrajectory(Drive drive, PathPlannerTrajectory trajectory) {
    return followTrajectory(drive, () -> trajectory);
  }

  /**
   * Creates a command that follows a trajectory, command ends when the trajectory
   * is finished
   */
  public static Command followTrajectory(
      Drive drive, Supplier<PathPlannerTrajectory> trajectorySupplier) {
    return Commands.startEnd(
        () -> drive.setTrajectory(trajectorySupplier.get()), drive::clearTrajectory)
        .until(drive::isTrajectoryCompleted);
  }

  /**
   * Returns whether robot has crossed x boundary, accounting for alliance flip
   *
   * @param xPosition         X position coordinate on blue side of field.
   * @param towardsCenterline Whether to wait until passed x coordinate towards
   *                          center line or away
   *                          from center line
   */
  public static boolean xCrossed(double xPosition, boolean towardsCenterline) {
    Pose2d robotPose = RobotState.getInstance().getTrajectorySetpoint();
    if (AllianceFlipUtil.shouldFlip()) {
      if (towardsCenterline) {
        return robotPose.getX() < FieldConstants.fieldLength - xPosition;
      } else {
        return robotPose.getX() > FieldConstants.fieldLength - xPosition;
      }
    } else {
      if (towardsCenterline) {
        return robotPose.getX() > xPosition;
      } else {
        return robotPose.getX() < xPosition;
      }
    }
  }

  /**
   * Command that waits for x boundary to be crossed. See
   * {@link #xCrossed(double, boolean)}
   */
  public static Command waitUntilXCrossed(double xPosition, boolean towardsCenterline) {
    return Commands.waitUntil(() -> xCrossed(xPosition, towardsCenterline));
  }

  /**
   * Returns whether robot has crossed y boundary, accounting for alliance flip
   *
   * @param yPosition         Y position coordinate on blue side of field.
   * @param towardsCenterline Whether to wait until passed y coordinate towards
   *                          center line or away
   *                          from center line
   */
  public static boolean yCrossed(double yPosition, boolean towardsCenterline) {
    Pose2d robotPose = RobotState.getInstance().getTrajectorySetpoint();
    if (AllianceFlipUtil.shouldFlip()) {
      if (towardsCenterline) {
        return robotPose.getY() < FieldConstants.fieldWidth - yPosition;
      } else {
        return robotPose.getY() > FieldConstants.fieldWidth - yPosition;
      }
    } else {
      if (towardsCenterline) {
        return robotPose.getY() > yPosition;
      } else {
        return robotPose.getY() < yPosition;
      }
    }
  }

  /**
   * Command that waits for y boundary to be crossed. See
   * {@link #yCrossed(double, boolean)}
   */
  public static Command waitUntilYCrossed(double yPosition, boolean towardsCenterline) {
    return Commands.waitUntil(() -> yCrossed(yPosition, towardsCenterline));
  }

  public static Command alertCommand(String alert) { // TODO: impl with alerts
    return Commands.runOnce(() -> System.out.println(alert));
  }

  public static Command scoreCommand(Superstructure superstructure) {
    return scoreCommand(superstructure, false);
  }

  public static Command scoreCommand(Superstructure superstructure, boolean isLeftL1) {
    return superstructure.scoreCommand(isLeftL1)
        .alongWith(Commands.waitSeconds(scoreWaitTime))
        .withName("Score")
        .alongWith(alertCommand("Scoring"));
  }

  public static Command driveWithSuperstructureCommand(Drive drive, Superstructure superstructure,
      PathPlannerTrajectory traj, Superstructure.Goal goal, double delay) {
    return followTrajectory(drive, traj)
        .deadlineFor(
            Commands.waitSeconds(delay)
                .andThen(superstructure.setGoalCommand(goal)))
        .withName("DriveWithSuperstructureGoal")
        .alongWith(alertCommand("Driving with Superstructure Goal " + goal));
  }

  public static Command driveAndScoreCommand(Drive drive, Superstructure superstructure,
      PathPlannerTrajectory traj, Superstructure.Goal goal, double delay) {
    return driveWithSuperstructureCommand(drive, superstructure, traj, goal, delay)
        .andThen(scoreCommand(superstructure));
  }

  public static Command driveAndScoreL1Command(Drive drive, Superstructure superstructure,
      PathPlannerTrajectory traj, double delay, boolean isLeftL1) {
    return driveWithSuperstructureCommand(drive, superstructure, traj, Superstructure.Goal.L1, delay)
        .andThen(scoreCommand(superstructure, isLeftL1));
  }

  public static Command driveAndIntakeCommand(Drive drive, Superstructure superstructure, PathPlannerTrajectory traj) {
    return superstructure.setGoalCommand(Superstructure.Goal.INTAKE)
        .raceWith(followTrajectory(drive, traj)
            .andThen(Commands.waitSeconds(intakeWaitTime)))
        .withName("DriveAndIntake")
        .alongWith(alertCommand("Driving and Intaking"));
  }
}
