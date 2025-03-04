package frc.team4276.frc2025.subsystems.drive.controllers;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Timer;
import frc.team4276.frc2025.RobotState;
import frc.team4276.util.dashboard.LoggedTunableNumber;
import frc.team4276.util.dashboard.LoggedTunablePID;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class TrajectoryController { // TODO: impl and test choreo trajectory auto
  private final LoggedTunableNumber maxError =
      new LoggedTunableNumber("TrajectoryController/maxError", 0.75);

  private PathPlannerTrajectory traj;

  private double startTime = 0.0;
  private double timeOffset = 0.0;

  private boolean isFinished = true;

  private final LoggedTunablePID xController;
  private final LoggedTunablePID yController;
  private final LoggedTunablePID rotController;

  private List<Vector<N2>> moduleForces =
      List.of(
          VecBuilder.fill(0.0, 0.0),
          VecBuilder.fill(0.0, 0.0),
          VecBuilder.fill(0.0, 0.0),
          VecBuilder.fill(0.0, 0.0));
  private final double[] dummyForces = {0.0, 0.0, 0.0, 0.0};

  public TrajectoryController() {
    xController = new LoggedTunablePID(4.0, 0.0, 0.0, 0.1, "TrajectoryController/TranslationX");
    yController = new LoggedTunablePID(4.0, 0.0, 0.0, 0.1, "TrajectoryController/TranslationY");
    rotController =
        new LoggedTunablePID(3.0, 0.0, 0.0, Math.toRadians(1.0), "TrajectoryController/Rotation");

    rotController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void setTrajectory(PathPlannerTrajectory traj) {
    this.traj = traj;
    isFinished = false;
    startTime = Timer.getTimestamp();
    timeOffset = 0.0;
  }

  public ChassisSpeeds update(Pose2d currentPose) {
    if (getTrajectoryTime() > traj.getTotalTimeSeconds()) {
      isFinished = true;
    }

    var sampledState = traj.sample(getTrajectoryTime());

    if (sampledState.pose.getTranslation().getDistance(currentPose.getTranslation())
        > maxError.getAsDouble()) {
      timeOffset += 0.02;

      var dummyState = traj.sample(getTrajectoryTime());

      sampledState = new PathPlannerTrajectoryState();
      sampledState.timeSeconds = dummyState.timeSeconds;
      sampledState.pose = dummyState.pose;
      sampledState.heading = dummyState.heading;
      sampledState.feedforwards =
          new DriveFeedforwards(dummyForces, dummyForces, dummyForces, dummyForces, dummyForces);
    }

    RobotState.getInstance().setTrajectorySetpoint(sampledState.pose);

    moduleForces = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      moduleForces.add(
          VecBuilder.fill(
              sampledState.feedforwards.robotRelativeForcesXNewtons()[i],
              sampledState.feedforwards.robotRelativeForcesYNewtons()[i]));
    }

    double xError = sampledState.pose.getX() - currentPose.getTranslation().getX();
    double yError = sampledState.pose.getY() - currentPose.getTranslation().getY();
    double xFeedback = xController.calculate(0.0, xError);
    double yFeedback = yController.calculate(0.0, yError);
    double thetaError =
        MathUtil.angleModulus(
            sampledState.pose.getRotation().minus(currentPose.getRotation()).getRadians());
    double thetaFeedback = rotController.calculate(0.0, thetaError);

    ChassisSpeeds outputSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            sampledState.fieldSpeeds.vxMetersPerSecond + xFeedback,
            sampledState.fieldSpeeds.vyMetersPerSecond + yFeedback,
            sampledState.fieldSpeeds.omegaRadiansPerSecond + thetaFeedback,
            currentPose.getRotation());

    Logger.recordOutput("TrajectoryController/SetpointPose", sampledState.pose);
    Logger.recordOutput("TrajectoryController/SetpointSpeeds", sampledState.fieldSpeeds);
    Logger.recordOutput("TrajectoryController/OutputSpeeds", outputSpeeds);
    Logger.recordOutput("TrajectoryController/TranslationError", Math.hypot(xError, yError));
    Logger.recordOutput("TrajectoryController/RotationError", thetaError);

    return outputSpeeds;
  }

  private double getTrajectoryTime() {
    return Timer.getTimestamp() - startTime - timeOffset;
  }

  public void stopTraj() {
    isFinished = true;
  }

  @AutoLogOutput(key = "TrajectoryController/Finished")
  public boolean isFinished() {
    return isFinished;
  }

  public List<Vector<N2>> getModuleForces() {
    return moduleForces;
  }
}
