package frc.team4276.frc2025.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public boolean leaderMotorConnected = true;
    public boolean followerMotorConnected = true;

    public Rotation2d positionRads = new Rotation2d();
    public Rotation2d velocityRadsPerSec = new Rotation2d();

    public double[] appliedVolts = new double[] {};
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};
    public boolean absoluteEncoderConnected = true;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  /** Run to setpoint angle in radians */
  public default void runSetpoint(double setpointRads, double ff) {}

  /** Run to setpoint angle in radians */
  public default void runSetpoint(double setpointRads) {}

  /** Run motors at volts */
  public default void runVolts(double volts) {}

  /** Run motors at current */
  public default void runCurrent(double amps) {}

  /** Set brake mode enabled */
  public default void setBrakeMode(boolean enabled) {}

  /** Set PID values */
  public default void setPID(double p, double i, double d) {}

  /** Stops motors */
  public default void stop() {}
}