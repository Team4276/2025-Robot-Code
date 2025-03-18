package frc.team4276.frc2025.subsystems.algaefier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.subsystems.algaefier.arm.Arm;
import frc.team4276.frc2025.subsystems.algaefier.roller.Gripper;
import java.util.function.BooleanSupplier;

public class Algaefier extends SubsystemBase {
  public enum Goal {
    STOW,
    INTAKE,
    CHARACTERIZING
  }

  private Goal goal = Goal.STOW;
  private boolean hasAlgae = false;

  private final Arm arm;
  private final Gripper gripper;

  private double armCharacterizationInput = 0.0;

  public Algaefier(Arm arm, Gripper gripper) {
    this.arm = arm;
    this.gripper = gripper;
  }

  @Override
  public void periodic() {}

  public void setArmCoastOverride(BooleanSupplier override) {
    // arm.setCoastOverride(override);

    // arm.periodic();
    // gripper.periodic();
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  public Command setGoalCommand(Goal goal) {
    return startEnd(() -> setGoal(goal), () -> setGoal(Goal.STOW));
  }

  public void acceptCharacterizationInput(double input) {
    armCharacterizationInput = input;
    setGoal(Goal.CHARACTERIZING);
  }

  public double getFFCharacterizationVelocity() {
    return arm.getFFCharacterizationVelocity();
  }

  public void endCharacterizaton() {
    arm.endCharacterizaton();
  }
}
