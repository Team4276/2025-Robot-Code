package frc.team4276.frc2025.subsystems.drive.controllers;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.team4276.frc2025.Constants;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class TrajectoryController {
  private Trajectory<SwerveSample> trajectory;

  private double startTime = 0.0;
  private double timeOffset = 0.0;

  private boolean isFinished = true;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotationController;

  private final double[] dummyForces = {0.0, 0.0, 0.0, 0.0};

  public TrajectoryController() {
    xController =
        new PIDController(DriveConstants.autoTranslationKp, 0.0, DriveConstants.autoTranslationKd);
    yController =
        new PIDController(DriveConstants.autoTranslationKp, 0.0, DriveConstants.autoTranslationKd);
    rotationController =
        new PIDController(DriveConstants.autoRotationKp, 0.0, DriveConstants.autoRotationKd);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void setTrajectory(Trajectory<SwerveSample> traj) {
    trajectory = traj;
    isFinished = false;
    startTime = Timer.getTimestamp();
    timeOffset = 0.0;
  }

  public ChassisSpeeds update(Pose2d currentPose) {
    var sampledState = trajectory.sampleAt(getTrajectoryTime(), false);

    // Auto finished or interrupted
    if (sampledState.isEmpty() || getTrajectoryTime() > trajectory.getTotalTime()) {
      isFinished = true;
      return new ChassisSpeeds();
    }

    SwerveSample targetState = sampledState.get();

    if (targetState.getPose().getTranslation().getDistance(currentPose.getTranslation())
        > DriveConstants.autoMaxError) {
      timeOffset += Constants.kLooperDt;

      var dummyState = trajectory.sampleAt(getTrajectoryTime(), false);
      if (dummyState.isEmpty()) {
        isFinished = true;
        return new ChassisSpeeds();
      }

      targetState =
          new SwerveSample(
              dummyState.get().t,
              dummyState.get().x,
              dummyState.get().y,
              dummyState.get().heading,
              0.0,
              0.0,
              0.0,
              0.0,
              0.0,
              0.0,
              dummyForces,
              dummyForces);
    }

    RobotState.getInstance().setTrajectorySetpoint(targetState.getPose());

    double xError = targetState.x - currentPose.getTranslation().getX();
    double yError = targetState.y - currentPose.getTranslation().getY();
    double xFeedback = xController.calculate(0.0, xError);
    double yFeedback = yController.calculate(0.0, yError);
    double thetaFF = targetState.omega;
    double thetaError = targetState.heading - currentPose.getRotation().getRadians();
    double thetaFeedback = rotationController.calculate(0.0, thetaError);

    ChassisSpeeds outputSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            targetState.vx + xFeedback,
            targetState.vy + yFeedback,
            thetaFF + thetaFeedback,
            currentPose.getRotation());

    Logger.recordOutput("Trajectory/SetpointPose", targetState.getPose());
    Logger.recordOutput("Trajectory/SetpointSpeeds/vx", targetState.vx);
    Logger.recordOutput("Trajectory/SetpointSpeeds/vy", targetState.vy);
    Logger.recordOutput("Trajectory/SetpointSpeeds/omega", targetState.omega);
    Logger.recordOutput("Trajectory/OutputSpeeds", outputSpeeds);
    Logger.recordOutput("Trajectory/TranslationError", Math.hypot(xError, yError));
    Logger.recordOutput("Trajectory/RotationError", thetaError);

    return outputSpeeds;
  }

  private double getTrajectoryTime() {
    return Timer.getTimestamp() - startTime - timeOffset;
  }

  public void stopTraj() {
    isFinished = true;
  }

  @AutoLogOutput(key = "Trajectory/Finished")
  public boolean isFinished() {
    return isFinished;
  }
}
