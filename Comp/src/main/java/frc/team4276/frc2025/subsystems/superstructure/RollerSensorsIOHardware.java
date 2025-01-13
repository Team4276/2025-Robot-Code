package frc.team4276.frc2025.subsystems.superstructure;

import frc.team4276.frc2025.Ports;
import frc.team4276.util.drivers.BeamBreak;

public class RollerSensorsIOHardware implements RollerSensorsIO {
  private final BeamBreak beamBreak;

  public RollerSensorsIOHardware() {
    beamBreak = new BeamBreak(Ports.CORAL_BREAK);
  }

  @Override
  public void updateInputs(RollerSensorsIOInputs inputs) {
    beamBreak.update();

    inputs.read = beamBreak.get();
    inputs.tripped = beamBreak.wasTripped();
    inputs.cleared = beamBreak.wasCleared();
  }
}
