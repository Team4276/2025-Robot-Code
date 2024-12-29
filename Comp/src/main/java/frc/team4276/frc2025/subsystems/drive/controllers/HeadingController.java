package frc.team4276.frc2025.subsystems.drive.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.team4276.frc2025.subsystems.drive.DriveConstants;
import java.util.function.Supplier;

public class HeadingController {
  private PIDController controller;

  private Supplier<Rotation2d> targetHeadingSupplier;

  public HeadingController() {
    controller =
        new PIDController(DriveConstants.snapKp, DriveConstants.snapKi, DriveConstants.snapKd);

    controller.setTolerance(DriveConstants.snapPositionTolerance);
    controller.enableContinuousInput(-Math.PI, Math.PI);

    targetHeadingSupplier = () -> new Rotation2d();
  }

  public void setTarget(Supplier<Rotation2d> targetSupplier) {
    targetHeadingSupplier = targetSupplier;
  }

  public double update(double headingRadians) {
    double output = controller.calculate(headingRadians, targetHeadingSupplier.get().getRadians());

    return output;
  }
}
