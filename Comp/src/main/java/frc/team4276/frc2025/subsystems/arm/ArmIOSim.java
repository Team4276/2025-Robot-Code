package frc.team4276.frc2025.subsystems.arm;

import static frc.team4276.frc2025.subsystems.arm.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;

public class ArmIOSim implements ArmIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getNEO(2),
          gearRatio,
          1.0,
          length,
          Math.toRadians(50.0),
          Math.toRadians(135.0),
          false,
          Math.toRadians(90.0));

  private final PIDController controller;
  private final TrapezoidProfile profile;
  private TrapezoidProfile.State setpointState;
  private final double kg = 0 / (2 * Math.PI);
  private final double kv = DCMotor.getNEO(1).KvRadPerSecPerVolt / gearRatio;
  private double appliedVoltage = 0.0;

  private final ArmViz setpointViz;

  public ArmIOSim() {
    controller = new PIDController(30.0, 0.0, 0.0);

    profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVel, maxAccel));
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

    if (DriverStation.isDisabled()) {
      setpointState = new TrapezoidProfile.State(inputs.positionRads, inputs.velocityRadsPerSec);
    }
  }

  /** Run to setpoint angle in radians */
  @Override
  public void runSetpoint(double setpointRads, double ff) {
    setpointState =
        profile.calculate(0.02, setpointState, new TrapezoidProfile.State(setpointRads, 0.0));
    setpointViz.update(setpointState.position);
    runVolts(
        controller.calculate(sim.getAngleRads(), setpointState.position)
            + ff
            + (kv * setpointState.velocity)
            + (kg * Math.cos(setpointState.position)));
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
