package frc.team4276.frc2025.subsystems.drive.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.team4276.util.dashboard.LoggedTunablePID;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class HeadingController {
  private final LoggedTunablePID controller;

  private Supplier<Rotation2d> targetHeadingSupplier;

  public HeadingController() {
    controller =
        new LoggedTunablePID(3.0, 0.0, 0.1, Units.degreesToRadians(1.0), "HeadingController");

    controller.enableContinuousInput(-Math.PI, Math.PI);

    targetHeadingSupplier = () -> Rotation2d.kZero;
  }

  public void setTarget(Supplier<Rotation2d> targetSupplier) {
    targetHeadingSupplier = targetSupplier;
  }

  @AutoLogOutput(key = "HeadingController/Output")
  public double update(double headingRadians) {
    double target = targetHeadingSupplier.get().getRadians();
    double error =
        MathUtil.angleModulus(
            Rotation2d.fromRadians(headingRadians).minus(targetHeadingSupplier.get()).getRadians());

    Logger.recordOutput("HeadingController/TargetHeading", target);
    Logger.recordOutput("HeadingController/MeasuredHeading", headingRadians);
    Logger.recordOutput("HeadingController/Error", error);

    return controller.calculate(error, 0.0);
  }
}
