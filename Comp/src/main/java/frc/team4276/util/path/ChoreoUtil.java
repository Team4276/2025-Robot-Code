package frc.team4276.util.path;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.util.AllianceFlipUtil;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.List;

public class ChoreoUtil {
  private ChoreoUtil() {}

  // Assume swerve sample and if not then we screwed ig
  /** Loads and flips trajectory accordingly */
  @SuppressWarnings("unchecked")
  public static Trajectory<SwerveSample> getSwerveTrajectory(String name) {
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
  public static Trajectory<SwerveSample> getSwerveTrajectory(String name, int split) {
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
    try {
      var traj =
          PathPlannerPath.fromChoreoTrajectory(name)
              .generateTrajectory(
                  new ChassisSpeeds(),
                  new Rotation2d(),
                  new RobotConfig(0.0, 0.0, null, new Translation2d[] {}));

      return AllianceFlipUtil.shouldFlip() ? traj.flip() : traj;
    } catch (Exception e) {
      return new PathPlannerTrajectory(List.of());
    }
  }

  public static PathPlannerTrajectory getPathPlannerTrajectoryFromChoreo(String name, int split) {
    try {
      var traj =
          PathPlannerPath.fromChoreoTrajectory(name, split)
              .generateTrajectory(
                  new ChassisSpeeds(),
                  new Rotation2d(),
                  new RobotConfig(0.0, 0.0, null, new Translation2d[] {}));

      return AllianceFlipUtil.shouldFlip() ? traj.flip() : traj;
    } catch (Exception e) {
      return new PathPlannerTrajectory(List.of());
    }
  }
}
