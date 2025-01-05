package frc.team4276.frc2025.subsystems.flywheels;

import static frc.team4276.frc2025.subsystems.flywheels.FlywheelConstants.*;
import static frc.team4276.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import frc.team4276.frc2025.Ports;
import java.util.function.DoubleSupplier;

public class FlywheelIOSpark implements FlywheelIO {
  private final SparkMax topSpark;
  private final SparkMax botSpark;
  private final RelativeEncoder topEncoder;
  private final RelativeEncoder botEncoder;

  private final Debouncer topConnectedDebounce = new Debouncer(0.5);
  private final Debouncer botConnectedDebounce = new Debouncer(0.5);

  public FlywheelIOSpark() {
    topSpark = new SparkMax(Ports.FLYWHEEL_TOP, MotorType.kBrushless);
    botSpark = new SparkMax(Ports.FLYWHEEL_BOTTOM, MotorType.kBrushless);
    topEncoder = topSpark.getEncoder();
    botEncoder = botSpark.getEncoder();

    // Configure drive motor
    var topConfig = new SparkFlexConfig();
    topConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
    topConfig
        .encoder
        .positionConversionFactor(unitsPerRotation)
        .velocityConversionFactor(unitsPerRotation)
        .uvwMeasurementPeriod(measurementPeriod)
        .uvwAverageDepth(avgSamplingDepth);
    topConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / readFreq))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        topSpark,
        5,
        () ->
            topSpark.configure(
                topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(topSpark, 5, () -> topEncoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        topSpark,
        topEncoder::getPosition,
        (value) -> inputs.topPositionRads = Units.rotationsToRadians(value));
    ifOk(topSpark, topEncoder::getVelocity, (value) -> inputs.topVelocityRpm = value);
    ifOk(
        topSpark,
        new DoubleSupplier[] {topSpark::getAppliedOutput, topSpark::getBusVoltage},
        (values) -> inputs.topAppliedVolts = values[0] * values[1]);
    ifOk(topSpark, topSpark::getOutputCurrent, (value) -> inputs.topSupplyCurrentAmps = value);
    ifOk(topSpark, topSpark::getMotorTemperature, (value) -> inputs.topTempCelsius = value);
    inputs.topMotorConnected = topConnectedDebounce.calculate(!sparkStickyFault);

    sparkStickyFault = false;
    ifOk(
        botSpark,
        botEncoder::getPosition,
        (value) -> inputs.bottomPositionRads = Units.rotationsToRadians(value));
    ifOk(botSpark, botEncoder::getVelocity, (value) -> inputs.bottomVelocityRpm = value);
    ifOk(
        botSpark,
        new DoubleSupplier[] {botSpark::getAppliedOutput, botSpark::getBusVoltage},
        (values) -> inputs.bottomAppliedVolts = values[0] * values[1]);
    ifOk(botSpark, botSpark::getOutputCurrent, (value) -> inputs.bottomSupplyCurrentAmps = value);
    ifOk(botSpark, botSpark::getMotorTemperature, (value) -> inputs.bottomtTempCelsius = value);
    inputs.bottomMotorConnected = botConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void runVolts(double topVolts, double botVolts) {
    topSpark.setVoltage(topVolts);
    botSpark.setVoltage(botVolts);
  }

  @Override
  public void runCharacterizationTop(double voltage) {
    topSpark.setVoltage(voltage);
  }

  @Override
  public void runCharacterizationBottom(double voltage) {
    botSpark.setVoltage(voltage);
  }

  @Override
  public void runVelocity(double topFeedforward, double bottomFeedforward) {
    topSpark.setVoltage(topFeedforward);
    botSpark.setVoltage(bottomFeedforward);
  }

  @Override
  public void stop() {
    topSpark.stopMotor();
    botSpark.stopMotor();
  }
}
