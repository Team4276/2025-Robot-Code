package frc.team4276.frc2025.subsystems.arm;

import static frc.team4276.frc2025.subsystems.arm.ArmConstants.*;
import static frc.team4276.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;

public class ArmIOSparkMax implements ArmIO {
  private final SparkBase leaderSpark;
  private final AbsoluteEncoder absoluteEncoder;
  private final SparkClosedLoopController closedLoopController;

  // Connection debouncers
  private final Debouncer leaderConnectedDebounce = new Debouncer(0.5);

  public ArmIOSparkMax() {
    leaderSpark = new SparkMax(leaderId, MotorType.kBrushless);
    absoluteEncoder = leaderSpark.getAbsoluteEncoder();
    closedLoopController = leaderSpark.getClosedLoopController();

    // Configure lead motor
    tryUntilOk(
        leaderSpark,
        5,
        () -> leaderSpark.configure(
            leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Update leader inputs
    sparkStickyFault = false;
    ifOk(
        leaderSpark,
        absoluteEncoder::getPosition,
        (value) -> inputs.positionRads = new Rotation2d(value).minus(offset).getRadians());
    ifOk(
        leaderSpark,
        absoluteEncoder::getVelocity,
        (value) -> inputs.velocityRadsPerSec = value);
    ifOk(
        leaderSpark,
        new DoubleSupplier[] { leaderSpark::getAppliedOutput, leaderSpark::getBusVoltage },
        (values) -> inputs.appliedVolts[0] = values[0] * values[1]);
    ifOk(leaderSpark, leaderSpark::getOutputCurrent, (value) -> inputs.currentAmps[0] = value);
    ifOk(leaderSpark, leaderSpark::getMotorTemperature, (value) -> inputs.tempCelcius[0] = value);
    inputs.leaderMotorConnected = leaderConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void runSetpoint(double setpointRads, double ff) {
    closedLoopController.setReference(
        MathUtil.clamp(setpointRads, minInput, maxInput),
        ControlType.kPosition,
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
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    // TODO: impl
  }

  @Override
  public void setPID(double p, double i, double d) {
  }

  @Override
  public void stop() {
    leaderSpark.stopMotor();
  }
}
