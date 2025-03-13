package frc.team4276.frc2025.subsystems.climber;

import static frc.team4276.util.SparkUtil.ifOk;
import static frc.team4276.util.SparkUtil.tryUntilOk;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleSupplier;

public class ClimberIOSparkMax implements ClimberIO {
  private SparkMax whench;
  private SparkMax wheel;

  public ClimberIOSparkMax(
      int whenchID, int wheelID, int whenchCurrentLimit, int wheelCurrentLimit) {
    SparkMaxConfig whenchConfig = new SparkMaxConfig();
    SparkMaxConfig wheelConfig = new SparkMaxConfig();

    wheel = new SparkMax(wheelID, MotorType.kBrushless);
    wheelConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(wheelID);
    wheelConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
    tryUntilOk(
        wheel,
        5,
        () ->
            wheel.configure(
                wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    whench = new SparkMax(whenchID, MotorType.kBrushless);
    whenchConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(whenchCurrentLimit)
        .voltageCompensation(12.0);
    tryUntilOk(
        whench,
        5,
        () ->
            whench.configure(
                whenchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    ifOk(whench, whench::getOutputCurrent, (value) -> inputs.whenchSupplyCurrentAmps = value);
    ifOk(
        whench,
        new DoubleSupplier[] {whench::getAppliedOutput, whench::getBusVoltage},
        (values) -> inputs.appliedVoltageWhench = values[0] * values[1]);
    ifOk(whench, whench::getMotorTemperature, (value) -> inputs.whenchTempCelsius = value);

    ifOk(wheel, wheel::getOutputCurrent, (value) -> inputs.wheelsSupplyCurrentAmps = value);
    ifOk(
        wheel,
        new DoubleSupplier[] {wheel::getAppliedOutput, wheel::getBusVoltage},
        (values) -> inputs.appliedVoltageWheels = values[0] * values[1]);
    ifOk(wheel, wheel::getMotorTemperature, (value) -> inputs.wheelsTempCelsius = value);
  }

  @Override
  public void runWheelsAtVolts(double volts) {
    wheel.setVoltage(volts);
  }

  @Override
  public void runRunWhenchAtVolts(double volts) {
    whench.setVoltage(volts);
  }
}
