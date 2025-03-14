package frc.team4276.frc2025.subsystems.algaefier.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.team4276.frc2025.Ports;

public class ArmConstants {
  public static final int leaderId = Ports.ALGAEFIER_ARM;

  public static final boolean invertLeader = false;

  public static final int currentLimit = 40;

  public static final double encoderPositionFactor = 2 * Math.PI;
  public static final double encoderVelocityFactor = 2 * Math.PI;
  public static final boolean invertEncoder = false;

  public static final double kp = 0.0;
  public static final double ki = 0.0;
  public static final double kd = 0.0;

  public static final int readFreq = 50;

  public static final double minInput = Math.toRadians(90.0);
  public static final double maxInput = Math.toRadians(91.0);

  public static final Rotation2d offset = Rotation2d.fromDegrees(0.0);

  public static final Translation2d origin =
      new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0));
  public static final double gearRatio = 1.0;
  public static final double length = Units.inchesToMeters(10.0);
}
