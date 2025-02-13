package frc.team4276.frc2025.subsystems.superstructure.endeffector;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import static frc.team4276.util.SparkUtil.ifOk;
import static frc.team4276.util.SparkUtil.sparkStickyFault;
import static frc.team4276.util.SparkUtil.tryUntilOk;

public class EndEffectorIOSparkMax implements EndEffectorIO {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder  rightEncoder;
  private final Debouncer motorConnectedDebounce = new Debouncer(0.5);

  public EndEffectorIOSparkMax(int left_id, int right_id, int currentLimit, boolean invert, boolean brake) {
    leftMotor = new SparkMax(left_id, MotorType.kBrushless);
    rightMotor = new SparkMax(right_id, MotorType.kBrushless);

    var config = new SparkMaxConfig();
    config
        .smartCurrentLimit(currentLimit)
        .inverted(invert)
        .idleMode(brake ? IdleMode.kBrake : IdleMode.kBrake);
    config.signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        leftMotor,
        5,
        () -> leftMotor.configure(
            config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    config
        .smartCurrentLimit(currentLimit)
        .inverted(!invert)
        .idleMode(brake ? IdleMode.kBrake : IdleMode.kBrake);
    config.signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        rightMotor,
        5,
        () -> rightMotor.configure(
            config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();  
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        leftMotor,
        new DoubleSupplier[] { leftMotor::getAppliedOutput, leftMotor::getBusVoltage },
        (values) -> inputs.leftAppliedVoltage = values[0] * values[1]);
    ifOk(leftMotor, leftMotor::getOutputCurrent, (value) -> inputs.leftSupplyCurrentAmps = value);
    ifOk(leftMotor, leftMotor::getMotorTemperature, (value) -> inputs.leftTempCelsius = value);
    ifOk(leftMotor, leftEncoder::getVelocity, (value) -> inputs.LeftVelocity = value);
    inputs.leftConnected = motorConnectedDebounce.calculate(!sparkStickyFault);

    ifOk(
        rightMotor,

        new DoubleSupplier[] { rightMotor::getAppliedOutput, rightMotor::getBusVoltage },
        (values) -> inputs.rightAppliedVoltage = values[0] * values[1]);
    ifOk(rightMotor, rightMotor::getOutputCurrent, (value) -> inputs.rightSupplyCurrentAmps = value);
    ifOk(rightMotor, rightMotor::getMotorTemperature, (value) -> inputs.RightTempCelsius = value);
    ifOk(rightMotor, rightEncoder::getVelocity, (value) -> inputs.RightVelocity = value);

    inputs.rightConnected = motorConnectedDebounce.calculate(!sparkStickyFault);

  }

  @Override
  public void runVolts(double leftVolts, double rightVolts) {
    leftMotor.setVoltage(leftVolts);
    rightMotor.setVoltage(rightVolts);
  }

  @Override
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
