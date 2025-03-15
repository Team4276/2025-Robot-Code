package frc.team4276.frc2025.subsystems.hopper;

import edu.wpi.first.math.geometry.Translation3d;

public class HopperConstants {
  public static final boolean invertRight = true;
  public static final boolean invertleft = false;

  public static final int currentLimit = 40;

  public static final double encoderPositionFactor = 1.0;
  public static final double encoderVelocityFactor = 1.0 / 60.0;
  public static final boolean invertEncoder = false;

  public static final double kp = 0.001;
  public static final double ki = 0.0;
  public static final double kd = 0.0;

  public static final int readFreq = 50;

  public static final double minInput = Math.toRadians(90.0);
  public static final double maxInput = Math.toRadians(91.0);

  public static final Translation3d origin = Translation3d.kZero;
}
