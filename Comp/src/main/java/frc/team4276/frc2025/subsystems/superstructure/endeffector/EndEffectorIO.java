package frc.team4276.frc2025.subsystems.superstructure.endeffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  @AutoLog
  abstract class EndEffectorIOInputs {
    public boolean connected = true;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(EndEffectorIOInputs inputs) {}

  /** Run feeder at volts */
  default void runVolts(double volts) {}

  /** Stop feeder */
  default void stop() {}
}
