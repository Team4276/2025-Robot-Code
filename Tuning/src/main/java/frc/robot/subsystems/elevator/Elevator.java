package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Elevator {
  private final SparkMax leaderSpark;
  private final SparkMax followerSpark;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController closedLoopController;

  public Elevator() {

    leaderSpark = new SparkMax(leaderId, MotorType.kBrushless);
    followerSpark = new SparkMax(followerId, MotorType.kBrushless);
    encoder = leaderSpark.getEncoder();
    closedLoopController = leaderSpark.getClosedLoopController();

    // Configure lead motor
    var leaderConfig = new SparkMaxConfig();
    leaderConfig
        .inverted(invertLeader)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(12.0);
    leaderConfig
        .encoder
        .positionConversionFactor(encoderPositionFactor)
        .velocityConversionFactor(encoderVelocityFactor);
    leaderConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            kp, ki,
            kd, kff);
    leaderConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / readFreq))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    var limitSwitchConfig = new LimitSwitchConfig();
    limitSwitchConfig
        .forwardLimitSwitchEnabled(true)
        .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true)
        .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
    leaderConfig.limitSwitch.apply(limitSwitchConfig);

    // Configure follower motor
    var followerConfig = new SparkMaxConfig();
    followerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(12.0)
        .follow(leaderSpark, invertFollower);
    followerConfig
        .signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        leaderSpark,
        5,
        () ->
            leaderSpark.configure(
                leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        followerSpark,
        5,
        () ->
            followerSpark.configure(
                followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    encoder.setPosition(homePosition);
  }

  public void periodic(double input) {
    leaderSpark.setVoltage(input * 4.0);
  }
}
