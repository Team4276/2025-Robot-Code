package frc.team4276.frc2025.subsystems.arm;

import static frc.team4276.frc2025.subsystems.arm.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;

public class ArmIOSim implements ArmIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          gearRatio,
          0.1,
          length,
          Math.toRadians(50.0),
          maxInput,
          false,
          minInput);

  private final PIDController controller;
  private double appliedVoltage = 0.0;

  private final ArmViz setpointViz;

  public ArmIOSim() {
    controller = new PIDController(30.0, 0.0, 0.0);

    setpointViz = new ArmViz("Setpoint", Color.kRed);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    sim.update(0.02);

    inputs.positionRads = sim.getAngleRads();
    inputs.velocityRadsPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = new double[] {appliedVoltage};
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.tempCelcius = new double[] {0.0};
  }

  /** Run to setpoint angle in radians */
  @Override
  public void runSetpoint(double setpointRads, double ff) {
    setpointViz.update(setpointRads);

    runVolts(controller.calculate(sim.getAngleRads(), setpointRads) + ff);
  }

  /** Run to setpoint angle in radians */
  @Override
  public void runSetpoint(double setpointRads) {
    runSetpoint(setpointRads, 0.0);
  }

  /** Run motors at volts */
  @Override
  public void runVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  /** Run motors at current */
  @Override
  public void runCurrent(double amps) {}

  /** Set brake mode enabled */
  @Override
  public void setBrakeMode(boolean enabled) {}

  /** Set PID values */
  @Override
  public void setPID(double p, double i, double d) {}

  /** Stops motors */
  @Override
  public void stop() {
    appliedVoltage = 0.0;
    sim.setInputVoltage(appliedVoltage);
  }
}
