package frc.team4276.frc2025.subsystems.algaefier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.subsystems.algaefier.arm.Arm;
import frc.team4276.frc2025.subsystems.algaefier.roller.Gripper;

public class Algaefier extends SubsystemBase {
  private final Arm arm;
  private final Gripper gripper;

  public Algaefier(Arm arm, Gripper gripper) {
    this.arm = arm;
    this.gripper = gripper;
  }

  @Override
  public void periodic() {}
}
