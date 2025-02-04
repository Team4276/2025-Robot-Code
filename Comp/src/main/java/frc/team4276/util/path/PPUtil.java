package frc.team4276.util.path;

import static frc.team4276.frc2025.field.FieldConstants.*;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

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
    flipped.fieldSpeeds = mirrorLengthwise(state.fieldSpeeds);
    flipped.heading = mirrorLengthwise(state.heading);

    return flipped;
  }

  private static Translation2d mirrorLengthwise(Translation2d trans) {
    return new Translation2d(trans.getX(), fieldWidth - trans.getY());
  }

  private static Rotation2d mirrorLengthwise(Rotation2d trans) {
    return trans.unaryMinus();
  }

  private static Pose2d mirrorLengthwise(Pose2d pose) {
    return new Pose2d(mirrorLengthwise(pose.getTranslation()), mirrorLengthwise(pose.getRotation()));
  }

  private static ChassisSpeeds mirrorLengthwise(ChassisSpeeds speeds) {
    return new ChassisSpeeds(
        speeds.vxMetersPerSecond,
        -1.0 * speeds.vyMetersPerSecond,
        -1.0 * speeds.omegaRadiansPerSecond);
  }

}
