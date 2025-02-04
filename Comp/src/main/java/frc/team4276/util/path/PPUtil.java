package frc.team4276.util.path;

import static frc.team4276.frc2025.field.FieldConstants.*;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;

public class PPUtil {
  public static PathPlannerTrajectory mirrorLengthwise(PathPlannerTrajectory trajectory) {
    List<PathPlannerTrajectoryState> mirroredStates = new ArrayList<>(trajectory.getStates().size());
    for (var state : trajectory.getStates()) {
      mirroredStates.add(mirrorLengthwise(state));
    }
    return new PathPlannerTrajectory(mirroredStates, trajectory.getEvents());
  }

  public static PathPlannerTrajectoryState mirrorLengthwise(PathPlannerTrajectoryState state) {
    var flipped = new PathPlannerTrajectoryState();

    flipped.timeSeconds = state.timeSeconds;
    flipped.linearVelocity = state.linearVelocity;
    flipped.pose = mirrorLengthwise(state.pose);
    flipped.fieldSpeeds = FlippingUtil.flipFieldSpeeds(state.fieldSpeeds); // TODO: finish
    flipped.heading = FlippingUtil.flipFieldRotation(state.heading);

    return flipped;
  }

  private static Pose2d mirrorLengthwise(Pose2d pose) {
    return new Pose2d(pose.getX(), fieldWidth - pose.getY(), pose.getRotation().unaryMinus());
  }

}
