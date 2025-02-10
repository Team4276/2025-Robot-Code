package frc.team4276.frc2025.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(690.875958);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final Translation2d fieldCenter = new Translation2d(
      Units.inchesToMeters(345.437979),
      Units.inchesToMeters(158.5));

  public static class POIs {
    public Translation2d reefCenter = Translation2d.kZero;
    // starts at the right most post just under the 0 degree line and moves
    // counterclockwise around the reef
    public Pose2d[] reefAlign = new Pose2d[12];
    public Pose2d[] reefScore = new Pose2d[12];
  }

  public static final double alignOffset = 40; // inches
  public static final double scoringOffset = 31; // inches

  public static final Translation2d reefToLeftAlign = new Translation2d(
      Units.inchesToMeters(20.738196 + alignOffset), Units.inchesToMeters(-6.468853));
  public static final Translation2d reefToRightAlign = new Translation2d(
      Units.inchesToMeters(20.738196 + alignOffset), Units.inchesToMeters(6.468853));

  public static final Translation2d reefToLeftScore = new Translation2d(
      Units.inchesToMeters(20.738196 + scoringOffset), Units.inchesToMeters(-6.468853));
  public static final Translation2d reefToRightScore = new Translation2d(
      Units.inchesToMeters(20.738196 + scoringOffset), Units.inchesToMeters(6.468853));

  public static final POIs bluePOIs = new POIs();
  static {
    bluePOIs.reefCenter = fieldCenter.plus(new Translation2d(-4.284788, 0.0));
    bluePOIs.reefAlign[0] = new Pose2d(bluePOIs.reefCenter.plus(reefToLeftAlign), Rotation2d.fromDegrees(180.0));
    bluePOIs.reefAlign[1] = new Pose2d(bluePOIs.reefCenter.plus(reefToRightAlign), Rotation2d.fromDegrees(180.0));
    bluePOIs.reefAlign[2] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToLeftAlign.rotateBy(Rotation2d.fromDegrees(60.0))),
        Rotation2d.fromDegrees(240.0));
    bluePOIs.reefAlign[3] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToRightAlign.rotateBy(Rotation2d.fromDegrees(60.0))),
        Rotation2d.fromDegrees(240.0));
    bluePOIs.reefAlign[4] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToLeftAlign.rotateBy(Rotation2d.fromDegrees(120.0))),
        Rotation2d.fromDegrees(300.0));
    bluePOIs.reefAlign[5] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToRightAlign.rotateBy(Rotation2d.fromDegrees(120.0))),
        Rotation2d.fromDegrees(300.0));
    bluePOIs.reefAlign[6] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToLeftAlign.rotateBy(Rotation2d.fromDegrees(180.0))),
        Rotation2d.fromDegrees(0.0));
    bluePOIs.reefAlign[7] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToRightAlign.rotateBy(Rotation2d.fromDegrees(180.0))),
        Rotation2d.fromDegrees(0.0));
    bluePOIs.reefAlign[8] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToLeftAlign.rotateBy(Rotation2d.fromDegrees(240.0))),
        Rotation2d.fromDegrees(60.0));
    bluePOIs.reefAlign[9] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToRightAlign.rotateBy(Rotation2d.fromDegrees(240.0))),
        Rotation2d.fromDegrees(60.0));
    bluePOIs.reefAlign[10] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToLeftAlign.rotateBy(Rotation2d.fromDegrees(300.0))),
        Rotation2d.fromDegrees(120.0));
    bluePOIs.reefAlign[11] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToRightAlign.rotateBy(Rotation2d.fromDegrees(300.0))),
        Rotation2d.fromDegrees(120.0));

    bluePOIs.reefScore[0] = new Pose2d(bluePOIs.reefCenter.plus(reefToLeftScore), Rotation2d.fromDegrees(180.0));
    bluePOIs.reefScore[1] = new Pose2d(bluePOIs.reefCenter.plus(reefToRightScore), Rotation2d.fromDegrees(180.0));
    bluePOIs.reefScore[2] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToLeftScore.rotateBy(Rotation2d.fromDegrees(60.0))),
        Rotation2d.fromDegrees(240.0));
    bluePOIs.reefScore[3] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToRightScore.rotateBy(Rotation2d.fromDegrees(60.0))),
        Rotation2d.fromDegrees(240.0));
    bluePOIs.reefScore[4] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToLeftScore.rotateBy(Rotation2d.fromDegrees(120.0))),
        Rotation2d.fromDegrees(300.0));
    bluePOIs.reefScore[5] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToRightScore.rotateBy(Rotation2d.fromDegrees(120.0))),
        Rotation2d.fromDegrees(300.0));
    bluePOIs.reefScore[6] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToLeftScore.rotateBy(Rotation2d.fromDegrees(180.0))),
        Rotation2d.fromDegrees(0.0));
    bluePOIs.reefScore[7] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToRightScore.rotateBy(Rotation2d.fromDegrees(180.0))),
        Rotation2d.fromDegrees(0.0));
    bluePOIs.reefScore[8] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToLeftScore.rotateBy(Rotation2d.fromDegrees(240.0))),
        Rotation2d.fromDegrees(60.0));
    bluePOIs.reefScore[9] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToRightScore.rotateBy(Rotation2d.fromDegrees(240.0))),
        Rotation2d.fromDegrees(60.0));
    bluePOIs.reefScore[10] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToLeftScore.rotateBy(Rotation2d.fromDegrees(300.0))),
        Rotation2d.fromDegrees(120.0));
    bluePOIs.reefScore[11] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToRightScore.rotateBy(Rotation2d.fromDegrees(300.0))),
        Rotation2d.fromDegrees(120.0));
  }

  public static final POIs redPOIs = new POIs();
  static {
    redPOIs.reefCenter = fieldCenter.plus(new Translation2d(4.284793, 0.0));
    var distanceBetweenReefs = redPOIs.reefCenter.getDistance(bluePOIs.reefCenter);
    for (int i = 0; i < bluePOIs.reefAlign.length; i++) {
      redPOIs.reefAlign[i] = new Pose2d(bluePOIs.reefAlign[i].getX() + distanceBetweenReefs,
          bluePOIs.reefAlign[i].getY(), bluePOIs.reefAlign[i].getRotation());
      redPOIs.reefScore[i] = new Pose2d(bluePOIs.reefScore[i].getX() + distanceBetweenReefs,
          bluePOIs.reefScore[i].getY(), bluePOIs.reefScore[i].getRotation());
    }
  }
}
