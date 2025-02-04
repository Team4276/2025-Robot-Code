package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Ports;

public class ElevatorConstants {
  public static final int leaderId = Ports.ELEVATOR_LEADER;
  public static final int followerId = Ports.ELEVATOR_FOLLOWER;

  public static final boolean invertLeader = false;
  public static final boolean invertFollower = true;

  public static final int currentLimit = 50;

  public static final double encoderPositionFactor = 1.0;
  public static final double encoderVelocityFactor = 1.0 / 60.0;
  public static final boolean invertEncoder = true;

  public static final double kp = 0.0;
  public static final double ki = 0.0;
  public static final double kd = 0.0;
  public static final double kff = 0.0;

  public static final int readFreq = 50;

  public static final double minInput = 0.0;
  public static final double maxInput = 10.0;

  public static final double gearRatio = 25.0;
  public static final Translation2d origin = new Translation2d(0.0, 0.0);
  public static final double length = 1.0;

  public static final double homePosition = 0.0;
  public static final double maxPosition = 1.0;

  public static final double tolerance = 1.0;
}
