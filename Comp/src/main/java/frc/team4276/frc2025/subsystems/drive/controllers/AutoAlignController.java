package frc.team4276.frc2025.subsystems.drive.controllers;

import static frc.team4276.frc2025.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.team4276.util.dashboard.LoggedTunableNumber;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AutoAlignController {
  private final LoggedTunableNumber translationkP = new LoggedTunableNumber("AutoAlignController/Translation/kP", 1.0);
  private final LoggedTunableNumber translationkD = new LoggedTunableNumber("AutoAlignController/Translation/kD", 0.0);
  private final LoggedTunableNumber translationKTol = new LoggedTunableNumber(
      "AutoAlignController/Translation/Tolerance", 0.05);

  private final LoggedTunableNumber rotationkP = new LoggedTunableNumber("AutoAlignController/Rotation/kP", 0.5);
  private final LoggedTunableNumber rotationkD = new LoggedTunableNumber("AutoAlignController/Rotation/kD", 0.0);
  private final LoggedTunableNumber rotationKTol = new LoggedTunableNumber(
      "AutoAlignController/Rotation/ToleranceDegrees", 3.0);

  private final ProfiledPIDController translationController;
  private final ProfiledPIDController headingController;
  private final Timer translationToleranceTimer = new Timer();
  private final Timer headingToleranceTimer = new Timer();
  private final double toleranceTime = 0.5;

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
        new TrapezoidProfile.Constraints(maxSpeed, maxAccel));
    headingController = new ProfiledPIDController(
        rotationkP.getAsDouble(),
        0,
        rotationkD.getAsDouble(),
        new TrapezoidProfile.Constraints(maxAngularSpeed, maxAngularAccel));
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    translationController.setTolerance(translationKTol.getAsDouble());
    headingController.setTolerance(Math.toRadians(rotationKTol.getAsDouble()));

    translationToleranceTimer.restart();
    headingToleranceTimer.restart();
  }

  public void setSetpoint(Pose2d pose) {
    this.setpoint = pose;
    translationToleranceTimer.reset();
    headingToleranceTimer.reset();
    cancelX = false;
    cancelY = false;
    cancelTheta = false;
  }

  public void feedTeleopInput(double x, double y, double omega) {
    teleopDriveController.feedDriveInput(x, y, omega);
  }

  private ChassisSpeeds updateContoller(Pose2d currentPose) {
    translationController.setPID(translationkP.getAsDouble(), 0.0, translationkD.getAsDouble());
    translationController.setTolerance(translationKTol.getAsDouble());
    headingController.setPID(rotationkP.getAsDouble(), 0.0, rotationkD.getAsDouble());
    headingController.setTolerance(Math.toRadians(rotationKTol.getAsDouble()));

    Translation2d trans = setpoint.getTranslation().minus(currentPose.getTranslation());
    Translation2d linearOutput = Translation2d.kZero;
    if (trans.getNorm() > 1e-6) {
      linearOutput = new Translation2d(
          translationController.calculate(0.0, trans.getNorm()) +
              translationController.getSetpoint().velocity,
          trans.unaryMinus().getAngle());
    }

    if (!translationController.atGoal()) {
      translationToleranceTimer.reset();

    }

    double thetaError = MathUtil.angleModulus(setpoint.getRotation().minus(currentPose.getRotation()).getRadians());
    double omega = headingController.calculate(0.0, thetaError) + headingController.getSetpoint().velocity;

    if (!headingController.atGoal()) {
      headingToleranceTimer.reset();

    }

    distanceToGoal = setpoint.relativeTo(currentPose);

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
    return translationToleranceTimer.hasElapsed(toleranceTime) &&
        headingToleranceTimer.hasElapsed(toleranceTime);
  }

  public Pose2d distToGoal() {
    return distanceToGoal;
  }

  public boolean isHeadingAligned() {
    return headingToleranceTimer.hasElapsed(toleranceTime);
  }
}
