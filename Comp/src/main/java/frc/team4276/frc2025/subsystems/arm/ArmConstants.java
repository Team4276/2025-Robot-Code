package frc.team4276.frc2025.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.team4276.frc2025.Ports;

public class ArmConstants {
  public static final int leaderId = Ports.ALGAE_INTAKE_ARM;

  public static final boolean invertLeader = false;

  public static final int currentLimit = 40;

  public static final double encoderPositionFactor = 2 * Math.PI;
  public static final double encoderVelocityFactor = 2 * Math.PI;
  public static final boolean invertEncoder = false;

  public static final double kp = 1.0;
  public static final double ki = 0.0;
  public static final double kd = 0.0;

  public static final int readFreq = 50;

  public static final double minInput = Math.toRadians(65.0);
  public static final double maxInput = Math.toRadians(110.0);

  public static final Rotation2d offset =
      Rotation2d.fromDegrees(221).rotateBy(Rotation2d.kCW_90deg);

  public static final Translation2d origin =
      new Translation2d(Units.inchesToMeters(-8.25), Units.inchesToMeters(6.991000));
  public static final double gearRatio = 81.0;
  public static final double length = Units.inchesToMeters(10.939282); // to the bend
}
