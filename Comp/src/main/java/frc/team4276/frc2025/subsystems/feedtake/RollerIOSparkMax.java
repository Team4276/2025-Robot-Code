package frc.team4276.frc2025.subsystems.feedtake;

import frc.team4276.frc2025.Ports;

public class RollerIOSparkMax extends GenericRollerSystemIOSparkMax implements RollerIO {
  private static final int id = Ports.INTAKE;
  private static final int currentLimitAmps = 40;
  private static final boolean invert = false;
  private static final boolean brake = true;

  public RollerIOSparkMax() {
    super(id, currentLimitAmps, invert, brake);
  }
}
