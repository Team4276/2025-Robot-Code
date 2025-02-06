package frc.robot.subsystems.endeffector;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class EndEffector {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;

  public EndEffector(int left_id, int right_id, int currentLimit, boolean invert, boolean brake) {
    leftMotor = new SparkMax(left_id, MotorType.kBrushless);
    rightMotor = new SparkMax(right_id, MotorType.kBrushless);

    var config = new SparkMaxConfig();
    config
        .smartCurrentLimit(currentLimit)
        .inverted(invert)
        .idleMode(brake ? IdleMode.kBrake : IdleMode.kBrake);
    config.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

    tryUntilOk(
        leftMotor,
        5,
        () ->
            leftMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    config.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
    tryUntilOk(
        rightMotor,
        5,
        () ->
            rightMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void periodic(boolean run) {
    leftMotor.setVoltage(run ? 3.0 : 0.0);
    rightMotor.setVoltage(run ? -10.0 : 0.0);
  }
}
