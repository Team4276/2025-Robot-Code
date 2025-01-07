package frc.team4276.frc2025.field;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(690.875958);
  public static final double fieldWidth = Units.inchesToMeters(317.000000);
  public static final Translation2d fieldCenter = new Translation2d(
      Units.inchesToMeters(345.437979),
      Units.inchesToMeters(158.5));

  public static class POIs {
    public Translation2d reefCenter = new Translation2d();
    // starts at the right most post just under the 0 degree line and moves counterclockwise around the reef
    public Translation2d[] reefScoring = new Translation2d[12];
  }

  public static final double scoringOffset = 40; // inches

  public static final Translation2d reefToLeftScoring = new Translation2d(Units.inchesToMeters(20.738196 + scoringOffset), Units.inchesToMeters(-6.468853));
  public static final Translation2d reefToRightScoring = new Translation2d(Units.inchesToMeters(20.738196 + scoringOffset), Units.inchesToMeters(6.468853));

  public static final POIs bluePOIs = new POIs();
  static {
    bluePOIs.reefCenter = fieldCenter.plus(new Translation2d(-4.284788, 0.0));
    bluePOIs.reefScoring[0] = bluePOIs.reefCenter.plus(reefToLeftScoring);
    bluePOIs.reefScoring[1] = bluePOIs.reefCenter.plus(reefToRightScoring);
    bluePOIs.reefScoring[2] = bluePOIs.reefCenter.plus(reefToLeftScoring.rotateBy(Rotation2d.fromDegrees(60.0)));
    bluePOIs.reefScoring[3] = bluePOIs.reefCenter.plus(reefToRightScoring.rotateBy(Rotation2d.fromDegrees(60.0)));
    bluePOIs.reefScoring[4] = bluePOIs.reefCenter.plus(reefToLeftScoring.rotateBy(Rotation2d.fromDegrees(120.0)));
    bluePOIs.reefScoring[5] = bluePOIs.reefCenter.plus(reefToRightScoring.rotateBy(Rotation2d.fromDegrees(120.0)));
    bluePOIs.reefScoring[6] = bluePOIs.reefCenter.plus(reefToLeftScoring.rotateBy(Rotation2d.fromDegrees(180.0)));
    bluePOIs.reefScoring[7] = bluePOIs.reefCenter.plus(reefToRightScoring.rotateBy(Rotation2d.fromDegrees(180.0)));
    bluePOIs.reefScoring[8] = bluePOIs.reefCenter.plus(reefToLeftScoring.rotateBy(Rotation2d.fromDegrees(240.0)));
    bluePOIs.reefScoring[9] = bluePOIs.reefCenter.plus(reefToRightScoring.rotateBy(Rotation2d.fromDegrees(240.0)));
    bluePOIs.reefScoring[10] = bluePOIs.reefCenter.plus(reefToLeftScoring.rotateBy(Rotation2d.fromDegrees(300.0)));
    bluePOIs.reefScoring[11] = bluePOIs.reefCenter.plus(reefToRightScoring.rotateBy(Rotation2d.fromDegrees(300.0)));
  }

  public static final POIs redPOIs = new POIs();
  static {
    redPOIs.reefCenter = fieldCenter.plus(new Translation2d(4.284793, 0.0));
    redPOIs.reefScoring[0] = redPOIs.reefCenter.plus(reefToLeftScoring);
    redPOIs.reefScoring[1] = redPOIs.reefCenter.plus(reefToRightScoring);
    redPOIs.reefScoring[2] = redPOIs.reefCenter.plus(reefToLeftScoring.rotateBy(Rotation2d.fromDegrees(60.0)));
    redPOIs.reefScoring[3] = redPOIs.reefCenter.plus(reefToRightScoring.rotateBy(Rotation2d.fromDegrees(60.0)));
    redPOIs.reefScoring[4] = redPOIs.reefCenter.plus(reefToLeftScoring.rotateBy(Rotation2d.fromDegrees(120.0)));
    redPOIs.reefScoring[5] = redPOIs.reefCenter.plus(reefToRightScoring.rotateBy(Rotation2d.fromDegrees(120.0)));
    redPOIs.reefScoring[6] = redPOIs.reefCenter.plus(reefToLeftScoring.rotateBy(Rotation2d.fromDegrees(180.0)));
    redPOIs.reefScoring[7] = redPOIs.reefCenter.plus(reefToRightScoring.rotateBy(Rotation2d.fromDegrees(180.0)));
    redPOIs.reefScoring[8] = redPOIs.reefCenter.plus(reefToLeftScoring.rotateBy(Rotation2d.fromDegrees(240.0)));
    redPOIs.reefScoring[9] = redPOIs.reefCenter.plus(reefToRightScoring.rotateBy(Rotation2d.fromDegrees(240.0)));
    redPOIs.reefScoring[10] = redPOIs.reefCenter.plus(reefToLeftScoring.rotateBy(Rotation2d.fromDegrees(300.0)));
    redPOIs.reefScoring[11] = redPOIs.reefCenter.plus(reefToRightScoring.rotateBy(Rotation2d.fromDegrees(300.0)));
  }
}
