package frc.team4276.frc2025.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSensorsIO {
  @AutoLog
  class RollerSensorsIOInputs {
    boolean read = false;
    boolean tripped = false;
    boolean cleared = false;
  }

  default void updateInputs(RollerSensorsIOInputs inputs) {}
}
