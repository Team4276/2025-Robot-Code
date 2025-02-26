package frc.team4276.frc2025.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.team4276.util.AllianceFlipUtil;

public class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(690.875958);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final Translation2d fieldCenter = new Translation2d(
      Units.inchesToMeters(345.437979),
      Units.inchesToMeters(158.5));

  public static final double reefToFieldCenter = 4.284788;

  public static final Pose2d blueReefCenter = new Pose2d(
      fieldCenter.minus(new Translation2d(reefToFieldCenter, 0.0)), Rotation2d.kZero);

  public static final double alignOffset = Units.inchesToMeters(9.0);
  public static final double scoringOffset = Units.inchesToMeters(31.0);
  public static final double reefCenterToTag = Units.inchesToMeters(20.738196);
  public static final double tagToReef = Units.inchesToMeters(6.468853);

  public static final Translation2d reefToLeftAlign = new Translation2d(
      reefCenterToTag + scoringOffset + alignOffset, -tagToReef);
  public static final Translation2d reefToRightAlign = reefToLeftAlign.plus(new Translation2d(0.0, 2.0 * tagToReef));

  public static final Translation2d reefToLeftScore = new Translation2d(
      reefCenterToTag + scoringOffset, -tagToReef);
  public static final Translation2d reefToRightScore = reefToLeftScore.plus(new Translation2d(0.0, 2.0 * tagToReef));

  public static final Pose2d[] blueReefToScore = new Pose2d[12];
  public static final Pose2d[] blueReefToAlign = new Pose2d[12];
  public static final Pose2d[] redReefToScore = new Pose2d[12];
  public static final Pose2d[] redReefToAlign = new Pose2d[12];
  static {
    for (int i = 0; i < 12; i++) {
      var angle = Rotation2d.fromDegrees(i * 60);
      blueReefToScore[i] = blueReefCenter.plus(new Transform2d(reefToLeftScore.rotateBy(angle), angle));
      blueReefToAlign[i] = blueReefCenter.plus(new Transform2d(reefToLeftAlign.rotateBy(angle), angle));
      redReefToScore[i] = AllianceFlipUtil.apply(blueReefToScore[i]);
      redReefToAlign[i] = AllianceFlipUtil.apply(blueReefToAlign[i]);
      i++;
      blueReefToScore[i] = blueReefCenter.plus(new Transform2d(reefToRightScore.rotateBy(angle), angle));
      blueReefToAlign[i] = blueReefCenter.plus(new Transform2d(reefToRightAlign.rotateBy(angle), angle));
      redReefToScore[i] = AllianceFlipUtil.apply(blueReefToScore[i]);
      redReefToAlign[i] = AllianceFlipUtil.apply(blueReefToAlign[i]);
    }
  }

  public enum Reef {
    // A(),
    // B,
    // C,
    // D,
    // E,
    // F,
    // G,
    // H,
    // I,
    // J,
    // K,
    // L;
    ;
    private final Pose2d score;
    private final Pose2d align;

    private Reef(Pose2d score, Pose2d align) {
      this.score = score;
      this.align = align;
    }

  }
}
