package frc.team4276.frc2025.subsystems.feedtake;

import frc.team4276.frc2025.Ports;
import frc.team4276.util.drivers.BeamBreak;

public class RollerSensorsIOHardware implements RollerSensorsIO {
  private final BeamBreak frontTrigger;
  private final BeamBreak backTrigger;

  public RollerSensorsIOHardware() {
    frontTrigger = new BeamBreak(Ports.BEAM_FRONT);
    backTrigger = new BeamBreak(Ports.BEAM_BACK);
  }

  @Override
  public void updateInputs(RollerSensorsIOInputs inputs) {
    frontTrigger.update();
    backTrigger.update();

    inputs.front = frontTrigger.get();
    inputs.frontTripped = frontTrigger.wasTripped();
    inputs.frontCleared = frontTrigger.wasCleared();

    inputs.back = backTrigger.get();
    inputs.backTripped = backTrigger.wasTripped();
    inputs.backCleared = backTrigger.wasCleared();
  }
}
