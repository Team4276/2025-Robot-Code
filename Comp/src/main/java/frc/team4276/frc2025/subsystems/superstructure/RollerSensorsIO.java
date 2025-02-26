package frc.team4276.frc2025.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSensorsIO {
  @AutoLog
  class RollerSensorsIOInputs {
    boolean frontRead = false;
    boolean frontTripped = false;
    boolean frontCleared = false;

    boolean backRead = false;
    boolean backTripped = false;
    boolean backCleared = false;
  }

  default void updateInputs(RollerSensorsIOInputs inputs) {}
}
