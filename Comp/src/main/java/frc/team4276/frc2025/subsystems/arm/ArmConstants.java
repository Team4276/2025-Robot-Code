package frc.team4276.frc2025.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.team4276.frc2025.Ports;

public class ArmConstants {
  public static final int leaderId = Ports.ALGAE_INTAKE_ARM;

  public static final boolean invertLeader = false;

  public static final int currentLimit = 50;

  public static final double encoderPositionFactor = 2 * Math.PI;
  public static final double encoderVelocityFactor = 2 * Math.PI / 60;
  public static final boolean invertEncoder = true;

  public static final double kp = 0.0;
  public static final double ki = 0.0;
  public static final double kd = 0.0;

  public static final int readFreq = 50;

  public static final double minInput = Math.toRadians(70.0);
  public static final double maxInput = Math.toRadians(160.0);

  public static final Rotation2d offset = Rotation2d.fromDegrees(0.0);

  public static final Translation2d origin = new Translation2d(-0.026301, 0.155575);
  public static final double gearRatio = 45.0;
  public static final double length = Units.inchesToMeters(10.939282); // to the bend
}
