package frc.team4276.frc2025.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.team4276.frc2025.Ports;

public class ArmConstants {
  public static final int leaderId = Ports.ARM_MASTER;
  public static final int followerId = Ports.ARM_FOLLOWER;

  public static final boolean invertFollower = true;

  public static final double gearRatio = 1.0;
  public static final double length = 1.0;

  public static final int currentLimit = 50;

  public static final double encoderPositionFactor = 2 * Math.PI;
  public static final double encoderVelocityFactor = 2 * Math.PI;
  public static final boolean invertEncoder = true;

  public static final double kp = 0.0;
  public static final double ki = 0.0;
  public static final double kd = 0.0;
  public static final double kff = 0.0;

  public static final int readFreq = 50;

  public static final double minInput = Math.toRadians(50.0);
  public static final double maxInput = Math.toRadians(135.0);

  public static final Rotation2d offset = Rotation2d.fromDegrees(0.0);

  public static final double allowedClosedLoopError = 0.1;

  public static final double maxAccel = 1.0;
  public static final double maxVel = 1.0;
}