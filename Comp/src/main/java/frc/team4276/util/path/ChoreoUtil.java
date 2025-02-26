package frc.team4276.util.path;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.team4276.frc2025.subsystems.drive.DriveConstants;
import frc.team4276.util.AllianceFlipUtil;
import java.util.List;

public class ChoreoUtil {
  private ChoreoUtil() {}

  // Assume swerve sample and if not then we screwed ig
  /** Loads and flips trajectory accordingly */
  @SuppressWarnings("unchecked")
  public static Trajectory<SwerveSample> getChoreoSwerveTrajectory(String name) {
    var uncheckedTraj = Choreo.loadTrajectory(name);

    if (uncheckedTraj.isEmpty()) {
      System.out.println("Failed to load trajectory " + name);
      return new Trajectory<SwerveSample>(name, List.of(), List.of(), List.of());
    }

    var traj = (Trajectory<SwerveSample>) uncheckedTraj.get();

    return AllianceFlipUtil.shouldFlip() ? traj.flipped() : traj;
  }

  // Assume swerve sample and if not then we screwed ig
  /** Loads and flips trajectory accordingly */
  @SuppressWarnings("unchecked")
  public static Trajectory<SwerveSample> getChoreoSwerveTrajectory(String name, int split) {
    var uncheckedTraj = Choreo.loadTrajectory(name);
    try {
      var traj =
          (Trajectory<SwerveSample>) uncheckedTraj.orElseThrow().getSplit(split).orElseThrow();

      return AllianceFlipUtil.shouldFlip() ? traj.flipped() : traj;
    } catch (Exception e) {
      System.out.println("Failed to load split " + split + " of trajectory " + name);
      return new Trajectory<SwerveSample>(name, List.of(), List.of(), List.of());
    }
  }

  public static PathPlannerTrajectory getPathPlannerTrajectoryFromChoreo(String name) {
    return getPathPlannerTrajectoryFromChoreo(name, false);
  }

  public static PathPlannerTrajectory getPathPlannerTrajectoryFromChoreo(
      String name, boolean mirrorLengthwise) {
    try {
      var traj =
          PathPlannerPath.fromChoreoTrajectory(name)
              .generateTrajectory(
                  new ChassisSpeeds(), Rotation2d.kZero, DriveConstants.driveConfig);

      if (mirrorLengthwise) {
        traj = PPUtil.mirrorLengthwise(traj);
      }

      return AllianceFlipUtil.shouldFlip() ? traj.flip() : traj;
    } catch (Exception e) {
      System.out.println("Failed to load Choreo Trajectory from PPlib " + name);
      System.out.println(e);
      return new PathPlannerTrajectory(List.of(new PathPlannerTrajectoryState()));
    }
  }

  public static PathPlannerTrajectory getPathPlannerTrajectoryFromChoreo(String name, int split) {
    return getPathPlannerTrajectoryFromChoreo(name, false, split);
  }

  public static PathPlannerTrajectory getPathPlannerTrajectoryFromChoreo(
      String name, boolean mirrorLengthwise, int split) {
    try {
      var traj =
          PathPlannerPath.fromChoreoTrajectory(name, split)
              .generateTrajectory(
                  new ChassisSpeeds(), Rotation2d.kZero, DriveConstants.driveConfig);

      if (mirrorLengthwise) {
        traj = PPUtil.mirrorLengthwise(traj);
      }

      return AllianceFlipUtil.shouldFlip() ? traj.flip() : traj;
    } catch (Exception e) {
      System.out.println(
          "Failed to load split " + split + " of Choreo Trajectory from PPlib " + name);
      System.out.println(e);
      return new PathPlannerTrajectory(List.of(new PathPlannerTrajectoryState()));
    }
  }
}
