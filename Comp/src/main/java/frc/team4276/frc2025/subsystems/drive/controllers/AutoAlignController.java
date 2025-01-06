package frc.team4276.frc2025.subsystems.drive.controllers;

import static frc.team4276.frc2025.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AutoAlignController {
  private final ProfiledPIDController translationController;
  private final ProfiledPIDController headingController;
  private final Timer toleranceTimer = new Timer();
  private final double toleranceTime = 0.5;

  private Pose2d setpoint = new Pose2d();

  public AutoAlignController() {
    translationController =
        new ProfiledPIDController(
            autoAlignTranslationKp,
            0,
            autoAlignTranslationKd,
            new TrapezoidProfile.Constraints(maxSpeed, maxAccel));
    headingController =
        new ProfiledPIDController(
            autoAlignRotationKp,
            0,
            autoAlignRotationKd,
            new TrapezoidProfile.Constraints(maxAngularSpeed, maxAngularAccel));
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    translationController.setTolerance(autoAlignTranslationTol);
    headingController.setTolerance(autoAlignRotationTol);

    toleranceTimer.restart();
  }

  public void setSetpoint(Pose2d pose) {
    this.setpoint = pose;
    toleranceTimer.reset();
  }

  public ChassisSpeeds update(Pose2d currentPose) {
    Translation2d trans = currentPose.getTranslation().minus(setpoint.getTranslation());
    Translation2d linearOutput = new Translation2d();
    if (trans.getNorm() > 1e-6) {
      linearOutput =
          new Translation2d(
              translationController.calculate(trans.getNorm(), 0.0), trans.getAngle());
    }

    double thetaError = setpoint.getRotation().minus(currentPose.getRotation()).getRadians();
    double omega = headingController.calculate(thetaError, 0.0);

    if (!translationController.atGoal() || !headingController.atGoal()) {
      toleranceTimer.reset();
    }

    Logger.recordOutput("AutoAlign/DistanceMeasured", trans.getNorm());
    Logger.recordOutput("AutoAlign/DistanceSetpoint", translationController.getSetpoint().position);
    Logger.recordOutput("AutoAlign/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("AutoAlign/ThetaSetpoint", headingController.getSetpoint().position);
    Logger.recordOutput(
        "AutoAlign/SetpointPose",
        new Pose2d(linearOutput, new Rotation2d(headingController.getSetpoint().position)));

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        linearOutput.getX(), linearOutput.getY(), omega, currentPose.getRotation());
  }

  @AutoLogOutput(key = "AutoAlign/AtGoal")
  public boolean atGoal() {
    return toleranceTimer.hasElapsed(toleranceTime);
  }
}
