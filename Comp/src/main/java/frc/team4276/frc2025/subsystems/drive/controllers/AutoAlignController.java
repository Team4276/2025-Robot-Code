package frc.team4276.frc2025.subsystems.drive.controllers;

import static frc.team4276.frc2025.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.team4276.frc2025.Constants;
import frc.team4276.frc2025.RobotState;
import frc.team4276.util.dashboard.LoggedTunableNumber;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AutoAlignController {
  private final LoggedTunableNumber translationkP = new LoggedTunableNumber("AutoAlignController/Translation/kP", 3.0);
  private final LoggedTunableNumber translationkD = new LoggedTunableNumber("AutoAlignController/Translation/kD", 0.0);
  private final LoggedTunableNumber translationKTol = new LoggedTunableNumber(
      "AutoAlignController/Translation/Tolerance", 0.05);
  private final LoggedTunableNumber translationMaxVel = new LoggedTunableNumber(
      "AutoAlignController/Translation/maxVel", 3.0);
  private final LoggedTunableNumber translationMaxAccel = new LoggedTunableNumber(
      "AutoAlignController/Translation/maxAccel", 3.0);

  private final LoggedTunableNumber rotationkP = new LoggedTunableNumber("AutoAlignController/Rotation/kP", 2.0);
  private final LoggedTunableNumber rotationkD = new LoggedTunableNumber("AutoAlignController/Rotation/kD", 0.0);
  private final LoggedTunableNumber rotationKTol = new LoggedTunableNumber(
      "AutoAlignController/Rotation/ToleranceDegrees", 3.0);
  private final LoggedTunableNumber rotationMaxVel = new LoggedTunableNumber("AutoAlignController/Rotation/maxVel",
      Units.degreesToRadians(360));
  private final LoggedTunableNumber rotationMaxAccel = new LoggedTunableNumber("AutoAlignController/Rotation/maxAccel",
      6.0);

  private final ProfiledPIDController translationController;
  private final ProfiledPIDController rotationController;

  @AutoLogOutput(key = "AutoAlign/SetpointPose")
  private Pose2d setpoint = Pose2d.kZero;
  private Pose2d distanceToGoal = Pose2d.kZero;

  private TeleopDriveController teleopDriveController = new TeleopDriveController();

  private boolean cancelX = false;
  private boolean cancelY = false;
  private boolean cancelTheta = false;

  public AutoAlignController() {
    translationController = new ProfiledPIDController(
        translationkP.getAsDouble(),
        0,
        translationkD.getAsDouble(),
        new TrapezoidProfile.Constraints(translationMaxVel.getAsDouble(), translationMaxAccel.getAsDouble()));
    rotationController = new ProfiledPIDController(
        rotationkP.getAsDouble(),
        0,
        rotationkD.getAsDouble(),
        new TrapezoidProfile.Constraints(maxAngularSpeed, maxAngularAccel));
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    translationController.setTolerance(translationKTol.getAsDouble());
    rotationController.setTolerance(Math.toRadians(rotationKTol.getAsDouble()));
  }

  public void setSetpoint(Pose2d pose) {
    this.setpoint = pose;
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    ChassisSpeeds fieldVelocity = RobotState.getInstance().getFieldVelocity();
    Translation2d linearFieldVelocity = new Translation2d(fieldVelocity.vxMetersPerSecond,
        fieldVelocity.vyMetersPerSecond);
    rotationController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    translationController.reset(
        currentPose.getTranslation().getDistance(pose.getTranslation()),
        Math.min(
            0.0,
            -linearFieldVelocity
                .rotateBy(
                    pose
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    cancelX = false;
    cancelY = false;
    cancelTheta = false;
  }

  public void feedTeleopInput(double x, double y, double omega) {
    teleopDriveController.feedDriveInput(x, y, omega);
  }

  private ChassisSpeeds updateContoller(Pose2d currentPose) {
    if (Constants.isTuning) {
      translationController.setPID(translationkP.getAsDouble(), 0.0, translationkD.getAsDouble());
      translationController.setTolerance(translationKTol.getAsDouble());
      translationController.setConstraints(
          new TrapezoidProfile.Constraints(translationMaxVel.getAsDouble(), translationMaxAccel.getAsDouble()));
      rotationController.setPID(rotationkP.getAsDouble(), 0.0, rotationkD.getAsDouble());
      rotationController.setTolerance(Math.toRadians(rotationKTol.getAsDouble()));
      rotationController.setConstraints(
          new TrapezoidProfile.Constraints(rotationMaxVel.getAsDouble(), rotationMaxAccel.getAsDouble()));
    }

    Translation2d trans = currentPose.getTranslation().minus(setpoint.getTranslation());
    Translation2d linearOutput = new Translation2d(
      translationController.calculate(trans.getNorm(), 0.0) +
          translationController.getSetpoint().velocity,
      trans.getAngle());
    if (trans.getNorm() < translationController.getPositionTolerance()) {
      linearOutput = Translation2d.kZero;
    }

    double thetaError = MathUtil.angleModulus(currentPose.getRotation().minus(setpoint.getRotation()).getRadians());
    double omega = rotationController.calculate(thetaError, 0.0) + rotationController.getSetpoint().velocity;
    if (Math.abs(thetaError) < rotationController.getPositionTolerance()) {
      omega = 0.0;
    }

    distanceToGoal = setpoint.relativeTo(currentPose);

    Logger.recordOutput("AutoAlign/DistanceSetpoint", translationController.getSetpoint().position);
    Logger.recordOutput("AutoAlign/DistanceError", trans.getNorm());
    Logger.recordOutput("AutoAlign/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("AutoAlign/ThetaSetpoint", setpoint.getRotation().getRadians());
    Logger.recordOutput("AutoAlign/ThetaError", thetaError);

    return new ChassisSpeeds(linearOutput.getX(), linearOutput.getY(), omega);

  }

  @AutoLogOutput(key = "AutoAlign/Output")
  public ChassisSpeeds update(Pose2d currentPose) {
    ChassisSpeeds controllerSpeeds = updateContoller(currentPose);

    var teleopSpeeds = teleopDriveController.updateRaw(currentPose.getRotation());

    cancelX = cancelX || Math.abs(teleopSpeeds.vxMetersPerSecond) > 1e-6;
    cancelY = cancelY || Math.abs(teleopSpeeds.vyMetersPerSecond) > 1e-6;
    cancelTheta = cancelTheta || Math.abs(teleopSpeeds.omegaRadiansPerSecond) > 1e-6;

    Logger.recordOutput("AutoAlign/CancelX", cancelX);
    Logger.recordOutput("AutoAlign/CancelY", cancelY);
    Logger.recordOutput("AutoAlign/CancelTheta", cancelTheta);

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        cancelX ? teleopSpeeds.vxMetersPerSecond : controllerSpeeds.vxMetersPerSecond,
        cancelY ? teleopSpeeds.vyMetersPerSecond : controllerSpeeds.vyMetersPerSecond,
        cancelTheta ? teleopSpeeds.omegaRadiansPerSecond : controllerSpeeds.omegaRadiansPerSecond,
        currentPose.getRotation());
  }

  @AutoLogOutput(key = "AutoAlign/AtGoal")
  public boolean atGoal() {
    return translationController.atGoal() &&
        rotationController.atGoal();
  }

  public Pose2d distToGoal() {
    return distanceToGoal;
  }

  public boolean isHeadingAligned() {
    return rotationController.atGoal();
  }
}
