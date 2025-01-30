package frc.team4276.frc2025.subsystems.arm;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

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

  public static final SparkMaxConfig leaderConfig = new SparkMaxConfig();

  static {
    leaderConfig
        .inverted(invertLeader)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(12.0);
    leaderConfig.absoluteEncoder
        .inverted(invertEncoder)
        .positionConversionFactor(encoderPositionFactor)
        .velocityConversionFactor(encoderVelocityFactor)
        .zeroOffset(offset.getRotations())
        .averageDepth(2);
    leaderConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0.0, 2 * Math.PI)
        .pidf(
            kp, ki,
            kd, 0.0);
    leaderConfig.signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / readFreq))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
  }
}
