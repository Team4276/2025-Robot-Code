package frc.team4276.frc2025.subsystems.drive.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.team4276.frc2025.RobotState;
import frc.team4276.util.dashboard.LoggedTunableProfiledPID;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AutoAlignController {
  private final LoggedTunableProfiledPID driveController;
  private final LoggedTunableProfiledPID thetaController;

  @AutoLogOutput(key = "AutoAlignController/SetpointPose")
  private Pose2d setpoint = Pose2d.kZero;

  private Pose2d distanceToGoal = Pose2d.kZero;

  private TeleopDriveController teleopDriveController = new TeleopDriveController();

  private boolean useCancel = false;
  private boolean cancelX = false;
  private boolean cancelY = false;
  private boolean cancelTheta = false;

  public
  AutoAlignController() { // TODO: feed txtypose into auto align when auto aligning with reef (or
    // change
    // to command structure)
    driveController =
        new LoggedTunableProfiledPID(
            "AutoAlignController/Translation", 3.0, 0.0, 0.0, 0.01, 3.0, 3.0);
    thetaController =
        new LoggedTunableProfiledPID(
            "AutoAlignController/Rotation", 3.0, 0.0, 0.0, Units.degreesToRadians(1.0), 6.0, 3.0);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void setSetpoint(Pose2d pose) {
    this.setpoint = pose;
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    ChassisSpeeds fieldVelocity = RobotState.getInstance().getFieldVelocity();
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    thetaController.reset(0.0, 0.0);
    driveController.reset(
        currentPose.getTranslation().getDistance(pose.getTranslation()),
        Math.min(
            0.0,
            -linearFieldVelocity
                .rotateBy(
                    pose.getTranslation()
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
    Translation2d trans = currentPose.getTranslation().minus(setpoint.getTranslation());
    Translation2d linearOutput =
        new Translation2d(
            driveController.calculate(trans.getNorm(), 0.0)
                + driveController.getSetpoint().velocity,
            trans.getAngle());
    if (trans.getNorm() < driveController.getPositionTolerance()) {
      linearOutput = Translation2d.kZero;
    }

    double thetaError =
        MathUtil.angleModulus(currentPose.getRotation().minus(setpoint.getRotation()).getRadians());
    double omega =
        thetaController.calculate(thetaError, 0.0) + thetaController.getSetpoint().velocity;
    if (Math.abs(thetaError) < thetaController.getPositionTolerance()) {
      omega = 0.0;
    }

    distanceToGoal = setpoint.relativeTo(currentPose);

    Logger.recordOutput(
        "AutoAlignController/DistanceSetpoint", driveController.getSetpoint().position);
    Logger.recordOutput("AutoAlignController/DistanceError", trans.getNorm());
    Logger.recordOutput(
        "AutoAlignController/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("AutoAlignController/ThetaSetpoint", setpoint.getRotation().getRadians());
    Logger.recordOutput("AutoAlignController/ThetaError", thetaError);

    return new ChassisSpeeds(linearOutput.getX(), linearOutput.getY(), omega);
  }

  @AutoLogOutput(key = "AutoAlignController/Output")
  public ChassisSpeeds update(Pose2d currentPose) {
    ChassisSpeeds controllerSpeeds = updateContoller(currentPose);

    var teleopSpeeds = teleopDriveController.updateRaw(currentPose.getRotation());

    // cancelX = (useCancel ? cancelX : false) ||
    // Math.abs(teleopSpeeds.vxMetersPerSecond) > 1e-6;
    // cancelY = (useCancel ? cancelY : false) ||
    // Math.abs(teleopSpeeds.vyMetersPerSecond) > 1e-6;
    // cancelTheta =
    // (useCancel ? cancelTheta : false) ||
    // Math.abs(teleopSpeeds.omegaRadiansPerSecond) > 1e-6;

    Logger.recordOutput("AutoAlignController/CancelX", cancelX);
    Logger.recordOutput("AutoAlignController/CancelY", cancelY);
    Logger.recordOutput("AutoAlignController/CancelTheta", cancelTheta);

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        cancelX ? teleopSpeeds.vxMetersPerSecond : controllerSpeeds.vxMetersPerSecond,
        cancelY ? teleopSpeeds.vyMetersPerSecond : controllerSpeeds.vyMetersPerSecond,
        cancelTheta ? teleopSpeeds.omegaRadiansPerSecond : controllerSpeeds.omegaRadiansPerSecond,
        currentPose.getRotation());
  }

  @AutoLogOutput(key = "AutoAlignController/atGoal")
  public boolean atGoal() {
    return driveController.atGoal() && thetaController.atGoal();
  }

  public Pose2d distToGoal() {
    return distanceToGoal;
  }

  public boolean isHeadingAligned() {
    return thetaController.atGoal();
  }
}
