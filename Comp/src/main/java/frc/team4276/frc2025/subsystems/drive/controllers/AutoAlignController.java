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

public class AutoAlignController { // TODO: cleanup/improve dashboard; cleanup advantage scope; add test autos; add update displacer logic;
  private final LoggedTunableNumber driveKp = new LoggedTunableNumber("AutoAlignController/Translation/kP", 3.0);
  private final LoggedTunableNumber driveKd = new LoggedTunableNumber("AutoAlignController/Translation/kD", 0.0);
  private final LoggedTunableNumber driveKTol = new LoggedTunableNumber(
      "AutoAlignController/Translation/Tolerance", 0.05);
  private final LoggedTunableNumber driveMaxVel = new LoggedTunableNumber(
      "AutoAlignController/Translation/maxVel", 3.0);
  private final LoggedTunableNumber driveMaxAccel = new LoggedTunableNumber(
      "AutoAlignController/Translation/maxAccel", 3.0);

  private final LoggedTunableNumber thetakP = new LoggedTunableNumber("AutoAlignController/Rotation/kP", 1.0);
  private final LoggedTunableNumber thetakD = new LoggedTunableNumber("AutoAlignController/Rotation/kD", 0.0);
  private final LoggedTunableNumber thetaKTol = new LoggedTunableNumber(
      "AutoAlignController/Rotation/ToleranceDegrees", 3.0);
  private final LoggedTunableNumber thetaMaxVel = new LoggedTunableNumber("AutoAlignController/Rotation/maxVel",
      Units.degreesToRadians(360));
  private final LoggedTunableNumber thetaMaxAccel = new LoggedTunableNumber("AutoAlignController/Rotation/maxAccel",
      6.0);

  private final ProfiledPIDController driveController;
  private final ProfiledPIDController thetaController;

  @AutoLogOutput(key = "AutoAlign/SetpointPose")
  private Pose2d setpoint = Pose2d.kZero;
  private Pose2d distanceToGoal = Pose2d.kZero;

  private TeleopDriveController teleopDriveController = new TeleopDriveController();

  private boolean cancelX = false;
  private boolean cancelY = false;
  private boolean cancelTheta = false;

  public AutoAlignController() {
    driveController = new ProfiledPIDController(
        driveKp.getAsDouble(),
        0,
        driveKd.getAsDouble(),
        new TrapezoidProfile.Constraints(driveMaxVel.getAsDouble(), driveMaxAccel.getAsDouble()));
    thetaController = new ProfiledPIDController(
        thetakP.getAsDouble(),
        0,
        thetakD.getAsDouble(),
        new TrapezoidProfile.Constraints(maxAngularSpeed, maxAngularAccel));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    driveController.setTolerance(driveKTol.getAsDouble());
    thetaController.setTolerance(Math.toRadians(thetaKTol.getAsDouble()));
  }

  public void setSetpoint(Pose2d pose) {
    this.setpoint = pose;
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    ChassisSpeeds fieldVelocity = RobotState.getInstance().getFieldVelocity();
    Translation2d linearFieldVelocity = new Translation2d(fieldVelocity.vxMetersPerSecond,
        fieldVelocity.vyMetersPerSecond);
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    driveController.reset(
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
      driveController.setPID(driveKp.getAsDouble(), 0.0, driveKd.getAsDouble());
      driveController.setTolerance(driveKTol.getAsDouble());
      driveController.setConstraints(
          new TrapezoidProfile.Constraints(driveMaxVel.getAsDouble(), driveMaxAccel.getAsDouble()));
      thetaController.setPID(thetakP.getAsDouble(), 0.0, thetakD.getAsDouble());
      thetaController.setTolerance(Math.toRadians(thetaKTol.getAsDouble()));
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(thetaMaxVel.getAsDouble(), thetaMaxAccel.getAsDouble()));
    }

    Translation2d trans = currentPose.getTranslation().minus(setpoint.getTranslation());
    Translation2d linearOutput = new Translation2d(
      driveController.calculate(trans.getNorm(), 0.0) +
          driveController.getSetpoint().velocity,
      trans.getAngle());
    if (trans.getNorm() < driveController.getPositionTolerance()) {
      linearOutput = Translation2d.kZero;
    }

    double thetaError = MathUtil.angleModulus(currentPose.getRotation().minus(setpoint.getRotation()).getRadians());
    double omega = thetaController.calculate(thetaError, 0.0) + thetaController.getSetpoint().velocity;
    if (Math.abs(thetaError) < thetaController.getPositionTolerance()) {
      omega = 0.0;
    }

    distanceToGoal = setpoint.relativeTo(currentPose);

    Logger.recordOutput("AutoAlign/DistanceSetpoint", driveController.getSetpoint().position);
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
    return driveController.atGoal() &&
        thetaController.atGoal();
  }

  public Pose2d distToGoal() {
    return distanceToGoal;
  }

  public boolean isHeadingAligned() {
    return thetaController.atGoal();
  }
}
