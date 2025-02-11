package frc.team4276.frc2025.subsystems.drive.controllers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.team4276.frc2025.subsystems.drive.DriveConstants;
import frc.team4276.util.AllianceFlipUtil;

public class TeleopDriveController {
  private static final double LINEAR_VELOCITY_SCALAR = 0.25;
  private static final double ANGULAR_VELOCITY_SCALAR = 0.65;

  private double controllerX = 0.0;
  private double controllerY = 0.0;
  private double controllerOmega = 0.0;

  public void feedDriveInput(double x, double y, double omega) {
    controllerX = x;
    controllerY = y;
    controllerOmega = omega;
  }

  public ChassisSpeeds updateRaw(Rotation2d yaw) {
    double linearMagnitude = Math.hypot(controllerX, controllerY);

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    Translation2d linearVelocity = Translation2d.kZero;

    if (linearMagnitude > 1e-6) {
      linearVelocity = new Translation2d(linearMagnitude, new Rotation2d(controllerX, controllerY))
          .times(LINEAR_VELOCITY_SCALAR);
    }

    // Square rotation value for more precise control
    double omega = Math.copySign(controllerOmega * controllerOmega, controllerOmega);

    return new ChassisSpeeds(
        linearVelocity.getX() * DriveConstants.maxSpeed,
        linearVelocity.getY() * DriveConstants.maxSpeed,
        omega * DriveConstants.maxAngularSpeed * ANGULAR_VELOCITY_SCALAR);
  }

  public ChassisSpeeds update(Rotation2d yaw) {
    return ChassisSpeeds.fromFieldRelativeSpeeds(updateRaw(yaw),
        AllianceFlipUtil.apply(yaw));
  }
}
