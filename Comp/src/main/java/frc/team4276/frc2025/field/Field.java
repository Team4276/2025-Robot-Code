package frc.team4276.frc2025.field;

import edu.wpi.first.math.geometry.Translation2d;

public class Field {
  public static class POIs {
    public Translation2d kSpeakerCenter;
    public Translation2d kSpeakerSS;
    public Translation2d kSpeakerAS;
    public Translation2d kAmp;
    public Translation2d kStageMid;
    public Translation2d kStageSS;
    public Translation2d kStageAS;
    public Translation2d kBank;
  }

  public static class Blue {
    public static final POIs kPOIs = new POIs();

    static {
      kPOIs.kSpeakerCenter = new Translation2d(0.225, 5.55);
      kPOIs.kSpeakerSS = new Translation2d();
      kPOIs.kSpeakerAS = new Translation2d();
      kPOIs.kAmp = new Translation2d();
      kPOIs.kStageMid = new Translation2d();
      kPOIs.kStageSS = new Translation2d();
      kPOIs.kStageAS = new Translation2d();
      kPOIs.kBank = new Translation2d(2.0, 7.0);
    }
  }

  public static class Red {
    public static final POIs kPOIs = new POIs();

    static {
      kPOIs.kSpeakerCenter = new Translation2d(16.317, 5.55);
      kPOIs.kSpeakerSS = new Translation2d();
      kPOIs.kSpeakerAS = new Translation2d();
      kPOIs.kAmp = new Translation2d();
      kPOIs.kStageMid = new Translation2d();
      kPOIs.kStageSS = new Translation2d();
      kPOIs.kStageAS = new Translation2d();
      kPOIs.kBank = new Translation2d(15.292, 7.0);
    }
  }
}
