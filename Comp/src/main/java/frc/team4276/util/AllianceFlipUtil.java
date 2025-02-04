package frc.team4276.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4276.frc2025.Constants;
import frc.team4276.frc2025.Constants.Mode;
import frc.team4276.frc2025.field.FieldConstants;

public class AllianceFlipUtil {
  static {
    SmartDashboard.putBoolean("Sim/OverrideFlip", false);
  }

  private static boolean overrideFlip = true;

  public static double applyX(double x) {
    return shouldFlip() ? FieldConstants.fieldLength - x : x;
  }

  public static double applyY(double y) {
    return shouldFlip() ? FieldConstants.fieldWidth - y : y;
  }

  public static Translation2d apply(Translation2d translation) {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip()
        ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
        : pose;
  }

  /**
   * For SIM
   * true sets to blue alliance
   * 
   * @param shouldOverrideFlip
   */
  public static void overrideFlip(boolean shouldOverrideFlip) {
    overrideFlip = shouldOverrideFlip;
  }

  public static boolean shouldFlip() {
    overrideFlip = SmartDashboard.getBoolean("Sim/OverrideFlip", overrideFlip);

    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
        && (Constants.getMode() == Mode.SIM ? !overrideFlip : true);
  }
}