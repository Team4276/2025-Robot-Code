package frc.team4276.frc2025.subsystems.superstructure.elevator;

import static frc.team4276.frc2025.subsystems.superstructure.elevator.ElevatorConstants.*;
import static frc.team4276.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkBase leaderSpark;
  private final SparkBase followerSpark;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController closedLoopController;

  // Connection debouncers
  private final Debouncer leaderConnectedDebounce = new Debouncer(0.5);
  private final Debouncer followerConnectedDebounce = new Debouncer(0.5);

  public ElevatorIOSparkMax() {
    leaderSpark = new SparkMax(leaderId, MotorType.kBrushless);
    followerSpark = new SparkMax(followerId, MotorType.kBrushless);
    encoder = leaderSpark.getEncoder();
    closedLoopController = leaderSpark.getClosedLoopController();

    // Configure lead motor
    var leaderConfig = new SparkMaxConfig();
    leaderConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(12.0);
    leaderConfig.encoder
        .inverted(invertEncoder)
        .positionConversionFactor(encoderPositionFactor)
        .velocityConversionFactor(encoderVelocityFactor);
    leaderConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            kp, ki,
            kd, kff);
    leaderConfig.closedLoop.maxMotion
        .allowedClosedLoopError(allowedClosedLoopError)
        .maxAcceleration(maxAccel)
        .maxVelocity(maxAccel);
    leaderConfig.signals
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
    followerConfig.signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        leaderSpark,
        5,
        () -> leaderSpark.configure(
            leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        followerSpark,
        5,
        () -> followerSpark.configure(
            followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    encoder.setPosition(homePosition);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Update leader inputs
    sparkStickyFault = false;
    ifOk(
        leaderSpark,
        encoder::getPosition,
        (value) -> inputs.position = value);
    ifOk(
        leaderSpark,
        encoder::getVelocity,
        (value) -> inputs.velocity = value);
    ifOk(
        leaderSpark,
        new DoubleSupplier[] { leaderSpark::getAppliedOutput, leaderSpark::getBusVoltage },
        (values) -> inputs.appliedVolts[0] = values[0] * values[1]);
    ifOk(leaderSpark, leaderSpark::getOutputCurrent, (value) -> inputs.currentAmps[0] = value);
    ifOk(leaderSpark, leaderSpark::getMotorTemperature, (value) -> inputs.tempCelcius[0] = value);
    inputs.leaderMotorConnected = leaderConnectedDebounce.calculate(!sparkStickyFault);

    // Update follower inputs
    sparkStickyFault = false;
    ifOk(
        followerSpark,
        new DoubleSupplier[] { followerSpark::getAppliedOutput, followerSpark::getBusVoltage },
        (values) -> inputs.appliedVolts[1] = values[0] * values[1]);
    ifOk(followerSpark, followerSpark::getOutputCurrent, (value) -> inputs.currentAmps[1] = value);
    ifOk(
        followerSpark,
        followerSpark::getMotorTemperature,
        (value) -> inputs.tempCelcius[1] = value);
    inputs.followerMotorConnected = followerConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void runSetpoint(double setpointRads, double ff) {
    closedLoopController.setReference(
        MathUtil.clamp(setpointRads, minInput, maxInput),
        ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0,
        ff,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void runSetpoint(double setpointRads) {
    runSetpoint(setpointRads, 0.0);
  }

  @Override
  public void runVolts(double volts) {
    leaderSpark.setVoltage(volts);
  }

  @Override
  public void runCurrent(double amps) {
    // TODO: impl
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    // TODO: impl
  }

  @Override
  public void setPID(double p, double i, double d) {
    // TODO: impl
  }

  @Override
  public void setPosition(double position) {
    encoder.setPosition(position);
  }

  @Override
  public void stop() {
    leaderSpark.stopMotor();
    followerSpark.stopMotor();
  }
}
