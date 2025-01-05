package frc.team4276.frc2025.subsystems.flywheels;

public class FlywheelConstants {
  public static int currentLimit = 50;

  public static int avgSamplingDepth = 8;
  public static int measurementPeriod = 10;
  public static double unitsPerRotation = 1.0;

  public static double ksTop = 0.188;
  public static double kvTop = 0.002;
  public static double ksBot = 0.188;
  public static double kvBot = 0.002;

  public static final int normalShotRPM = 3500;
  public static final int ferryRPM = 3500;
  public static final int spinUpRPM = 2500;
  public static final int poopTopRPM = 3500;
  public static final int poopBotRPM = 3200;
  public static final int exhaustRPM = -1000;
  public static final int ampTopRPM = 150;
  public static final int ampBotRPM = 1450;

  public static final double tolerance = 300.0; // RPM

  public static final int readFreq = 50;
}
