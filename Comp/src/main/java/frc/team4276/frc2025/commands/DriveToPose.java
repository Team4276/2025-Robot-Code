package frc.team4276.frc2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.drive.Drive.DriveMode;
import frc.team4276.util.dashboard.LoggedTunableProfiledPID;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
  private static final LoggedTunableProfiledPID driveController;
  private static final LoggedTunableProfiledPID thetaController;

  static {
    driveController =
        new LoggedTunableProfiledPID("DriveToPose/Translation", 3.0, 0.0, 0.0, 0.01, 3.0, 3.0);
    thetaController =
        new LoggedTunableProfiledPID(
            "DriveToPose/Rotation", 3.0, 0.0, 0.0, Units.degreesToRadians(1.0), 6.0, 3.0);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private boolean isRunning = false;

  private Pose2d error = Pose2d.kZero;

  private final Drive drive;
  private final Supplier<Pose2d> target;

  public DriveToPose(Drive drive, Supplier<Pose2d> target) {
    this.drive = drive;
    this.target = target;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    ChassisSpeeds fieldVelocity = RobotState.getInstance().getFieldVelocity();
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    thetaController.reset(0.0, 0.0);
    driveController.reset(
        currentPose.getTranslation().getDistance(target.get().getTranslation()),
        Math.min(
            0.0,
            -linearFieldVelocity
                .rotateBy(
                    target
                        .get()
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
  }

  @Override
  public void execute() {
    isRunning = true;

    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();

    Translation2d trans = currentPose.getTranslation().minus(target.get().getTranslation());
    Translation2d linearOutput =
        new Translation2d(
            driveController.calculate(trans.getNorm(), 0.0)
                + driveController.getSetpoint().velocity,
            trans.getAngle());
    if (trans.getNorm() < driveController.getPositionTolerance()) {
      linearOutput = Translation2d.kZero;
    }

    double thetaError =
        MathUtil.angleModulus(
            currentPose.getRotation().minus(target.get().getRotation()).getRadians());
    double omega =
        thetaController.calculate(thetaError, 0.0) + thetaController.getSetpoint().velocity;
    if (Math.abs(thetaError) < thetaController.getPositionTolerance()) {
      omega = 0.0;
    }

    error = target.get().relativeTo(currentPose);

    Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
    Logger.recordOutput("DriveToPose/DistanceError", trans.getNorm());
    Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("DriveToPose/ThetaSetpoint", target.get().getRotation().getRadians());
    Logger.recordOutput("DriveToPose/ThetaError", thetaError);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearOutput.getX(), linearOutput.getY(), omega, currentPose.getRotation()),
        DriveMode.AUTO_ALIGN);
  }

  @AutoLogOutput(key = "DriveToPose/atGoal")
  public boolean atGoal() {
    return isRunning && driveController.atGoal() && thetaController.atGoal();
  }

  @AutoLogOutput(key = "DriveToPose/withinTol")
  public boolean withinTolerance(double translation, double heading) {
    return isRunning
        && Math.abs(error.getTranslation().getNorm()) < translation
        && Math.abs(error.getRotation().getRadians()) < heading;
  }

  public Pose2d distToGoal() {
    return error;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    isRunning = false;
    // Clear logs
    Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {});
  }
}
