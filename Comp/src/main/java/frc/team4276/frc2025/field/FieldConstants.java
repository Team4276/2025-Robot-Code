package frc.team4276.frc2025.field;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(651.223);
  public static final double fieldWidth = Units.inchesToMeters(323.277);
  public static final double wingX = Units.inchesToMeters(229.201);
  public static final double podiumX = Units.inchesToMeters(126.75);
  public static final double startingLineX = Units.inchesToMeters(74.111);

  public static class POIs {
    public Translation2d speakerCenter;
    public Translation2d speakerSS;
    public Translation2d speakerAS;
    public Translation2d amp;
    public Translation2d stageMid;
    public Translation2d stageSS;
    public Translation2d stageAS;
    public Translation2d bank;
  }

  public static final POIs bluePOIs = new POIs();

  static {
    bluePOIs.speakerCenter = new Translation2d(0.225, 5.55);
    bluePOIs.speakerSS = new Translation2d();
    bluePOIs.speakerAS = new Translation2d();
    bluePOIs.amp = new Translation2d();
    bluePOIs.stageMid = new Translation2d();
    bluePOIs.stageSS = new Translation2d();
    bluePOIs.stageAS = new Translation2d();
    bluePOIs.bank = new Translation2d(2.0, 7.0);
  }
  ;

  public static final POIs redPOIs = new POIs();

  static {
    redPOIs.speakerCenter = new Translation2d(16.317, 5.55);
    redPOIs.speakerSS = new Translation2d();
    redPOIs.speakerAS = new Translation2d();
    redPOIs.amp = new Translation2d();
    redPOIs.stageMid = new Translation2d();
    redPOIs.stageSS = new Translation2d();
    redPOIs.stageAS = new Translation2d();
    redPOIs.bank = new Translation2d(15.292, 7.0);
  }
  ;
}
