package frc.team4276.frc2025.subsystems.drive.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.team4276.util.dashboard.LoggedTunableNumber;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class HeadingController {
  private final LoggedTunableNumber kP = new LoggedTunableNumber("HeadingController/kP", 0.5);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("HeadingController/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("HeadingController/kD", 0.1);
  private static final LoggedTunableNumber toleranceDegrees = new LoggedTunableNumber(
      "HeadingController/ToleranceDegrees", 3.0);
  private PIDController controller;

  private Supplier<Rotation2d> targetHeadingSupplier;

  public HeadingController() {
    controller = new PIDController(kP.get(), kI.get(), kD.get());

    controller.setTolerance(Math.toRadians(toleranceDegrees.get()));
    controller.enableContinuousInput(-Math.PI, Math.PI);

    targetHeadingSupplier = () -> Rotation2d.kZero;
  }

  public void setTarget(Supplier<Rotation2d> targetSupplier) {
    targetHeadingSupplier = targetSupplier;
  }

  public double update(double headingRadians) {
    double target = targetHeadingSupplier.get().getRadians();
    double error = Rotation2d.fromRadians(headingRadians).minus(targetHeadingSupplier.get()).getRadians();

    // Update controller
    controller.setPID(kP.get(), kI.get(), kD.get());
    controller.setTolerance(Math.toRadians(toleranceDegrees.get()));

    double output = controller.calculate(error, 0.0);

    Logger.recordOutput("HeadingController/TargetHeading", target);
    Logger.recordOutput("HeadingController/MeasuredHeading", headingRadians);
    Logger.recordOutput("HeadingController/Error", error);

    return output;
  }
}
