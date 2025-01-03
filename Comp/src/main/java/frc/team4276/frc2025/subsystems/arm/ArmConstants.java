package frc.team4276.frc2025.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.team4276.frc2025.Ports;
import frc.team4276.util.feedforwards.FourbarFeedForward;

public class ArmConstants {
  public static final int leaderId = Ports.ARM_MASTER;
  public static final int followerId = Ports.ARM_FOLLOWER;

  public static final boolean invertFollower = true;

  public static final double gearRatio = 135.0;
  public static final Translation2d armOrigin = new Translation2d(-0.026301, 0.155575);
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

  public static final double maxAccel = 2.0;
  public static final double maxVel = 2.0;

  public static final FourbarFeedForward.FourbarFeedForwardConstants kFeedForwardConstants =
      new FourbarFeedForward.FourbarFeedForwardConstants();

  static {
    kFeedForwardConstants.kS = 0.14;

    kFeedForwardConstants.kMotorFreeSpeedRpm = 5676.0;
    kFeedForwardConstants.kGearRatio = 185.712;
    kFeedForwardConstants.kStallTorque = 3.28;
    kFeedForwardConstants.kMotorAmnt = 2;
    kFeedForwardConstants.kEfficiency = 0.3;

    kFeedForwardConstants.kBottomLength = Units.inchesToMeters(8.001578);
    kFeedForwardConstants.kMotorLegLength = Units.inchesToMeters(11.000000);
    kFeedForwardConstants.kTopLength = Units.inchesToMeters(12.000808);
    kFeedForwardConstants.kSupportLegLength = Units.inchesToMeters(9.750);

    kFeedForwardConstants.kMotorLegMass = Units.lbsToKilograms(0.86);
    kFeedForwardConstants.kTopMass = Units.lbsToKilograms(28.0);
    kFeedForwardConstants.kSupportLegMass = Units.lbsToKilograms(0.494);

    kFeedForwardConstants.kMotorToCom =
        new Translation2d(Units.inchesToMeters(5.5), Units.inchesToMeters(0));
    kFeedForwardConstants.kMotorLegToTopCom =
        new Translation2d(Units.inchesToMeters(4.799278), Units.inchesToMeters(1.121730));
    kFeedForwardConstants.kSupportToCom =
        new Translation2d(Units.inchesToMeters(5.076911), Units.inchesToMeters(1.096583));
  }
}
