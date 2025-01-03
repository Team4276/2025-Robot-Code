package frc.team4276.util.path;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.util.AllianceFlipUtil;
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
}
