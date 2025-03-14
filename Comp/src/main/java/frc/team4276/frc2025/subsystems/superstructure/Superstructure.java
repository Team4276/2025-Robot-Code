package frc.team4276.frc2025.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.subsystems.superstructure.elevator.Elevator;
import frc.team4276.frc2025.subsystems.superstructure.endeffector.EndEffector;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final Elevator elevator;
  private final EndEffector endeffector;

  private final RollerSensorsIO sensorsIO;
  private final RollerSensorsIOInputsAutoLogged sensorsInputs =
      new RollerSensorsIOInputsAutoLogged();

  private boolean wantScore = false;
  private boolean leftL1 = false;

  private boolean wantUnjam = false;

  public enum Goal {
    STOW,
    INTAKE,
    L1,
    L2,
    L3,
    LO_ALGAE,
    HI_ALGAE,
    UNJAM,
    SHUFFLE,
    CHARACTERIZING,
    CUSTOM
  }

  private Supplier<Goal> desiredGoal = () -> Goal.STOW;
  private Goal currentGoal = Goal.STOW;

  private double elevatorCharacterizationInput = 0.0;

  private boolean hasGrasped = false; // clean up this mess
  private boolean cleared2 = false;
  private boolean cleared3 = false;
  private boolean cleared4 = false;
  private boolean cleared5 = false;

  private boolean hasCoral = false;

  public Superstructure(
      Elevator elevator, EndEffector endeffector, RollerSensorsIO sensorsIO) {
    this.elevator = elevator;
    this.endeffector = endeffector;
    this.sensorsIO = sensorsIO;

    elevator.setCoastOverride(() -> false);

    setDefaultCommand(setGoalCommand(() -> Superstructure.Goal.STOW));
  }

  @Override
  public void periodic() {
    sensorsIO.updateInputs(sensorsInputs);
    Logger.processInputs("RollersSensors", sensorsInputs);

    if (desiredGoal.get() == Goal.STOW && currentGoal == Goal.INTAKE && hasGrasped && !cleared3) {
      // Continue intaking

    } else {
      currentGoal = desiredGoal.get();
    }

    if (wantUnjam) {
      currentGoal = Goal.UNJAM;
    }

    if (wantScore) {
      endeffector.setGoal(
          currentGoal == Goal.L1
              ? (leftL1 ? EndEffector.Goal.FAVOR_LEFT : EndEffector.Goal.FAVOR_RIGHT)
              : EndEffector.Goal.SCORE);
    } else {
      endeffector.setGoal(EndEffector.Goal.IDLE);
    }

    if (currentGoal != Goal.INTAKE) {
      hasGrasped = false;
      cleared2 = false;
      cleared3 = false;
    }

    if (currentGoal != Goal.SHUFFLE) {
      cleared4 = false;
      cleared5 = false;
    }

    switch (currentGoal) {
      case STOW:
        elevator.setGoal(Elevator.Goal.STOW);
        endeffector.setGoal(EndEffector.Goal.IDLE);

        break;

      case SHUFFLE:
        if (sensorsInputs.backTripped || cleared5) {
          cleared5 = true;
          endeffector.setGoal(EndEffector.Goal.IDLE);
        } else if (sensorsInputs.backCleared || cleared4) {
          cleared4 = true;
          endeffector.setGoal(EndEffector.Goal.REVERSE);
        } else {
          endeffector.setGoal(EndEffector.Goal.SLOINTAKE);
        }

        break;

      case UNJAM:
        elevator.setGoal(Elevator.Goal.UNJAM);
        endeffector.setGoal(EndEffector.Goal.IDLE);

        break;
      case INTAKE:
        elevator.setGoal(Elevator.Goal.INTAKE);
        if ((sensorsInputs.backTripped && cleared2) || cleared3) {
          cleared3 = true;
          hasCoral = sensorsInputs.frontRead;
          endeffector.setGoal(EndEffector.Goal.IDLE);
        } else if (sensorsInputs.backCleared || cleared2) {
          cleared2 = true;
          endeffector.setGoal(EndEffector.Goal.REVERSE);
        } else if (sensorsInputs.backTripped || hasGrasped) {
          hasGrasped = true;
          endeffector.setGoal(EndEffector.Goal.SLOINTAKE);
        } else {
          endeffector.setGoal(EndEffector.Goal.INTAKE);
        }

        break;

      case L1:
        elevator.setGoal(Elevator.Goal.L1);

        break;

      case L2:
        elevator.setGoal(Elevator.Goal.L2);
        elevator.requestHome();

        break;

      case L3:
        elevator.setGoal(Elevator.Goal.L3);
        elevator.requestHome();

        break;

      case LO_ALGAE:
        elevator.setGoal(Elevator.Goal.LO_ALGAE);
        endeffector.setGoal(EndEffector.Goal.IDLE);

        break;
      case HI_ALGAE:
        elevator.setGoal(Elevator.Goal.HI_ALGAE);
        endeffector.setGoal(EndEffector.Goal.IDLE);

        break;
      case CHARACTERIZING:
        elevator.runCharacterization(elevatorCharacterizationInput);
        endeffector.setGoal(endEffectorGoal);
        break;

      case CUSTOM:
        elevator.setGoal(Elevator.Goal.CUSTOM);

      default:
        break;
    }

    elevator.periodic();
    endeffector.periodic();

    Logger.recordOutput("Superstructure/DesiredGoal", desiredGoal.get());
    Logger.recordOutput("Superstructure/CurrentGoal", currentGoal);
    Logger.recordOutput("Superstructure/WantScore", wantScore);
    Logger.recordOutput("Superstructure/HasCoral", hasCoral);
  }

  public boolean atGoal() {
    return elevator.atGoal();
  }

  public void setGoal(Goal goal) {
    desiredGoal = () -> goal;
  }

  public void setGoal(Supplier<Goal> goal) {
    desiredGoal = goal;
  }

  public Goal getGoal() {
    return currentGoal;
  }

  public Command setGoalCommand(Goal goal) {
    return setGoalCommand(() -> goal);
  }

  public Command setGoalCommand(Supplier<Goal> goal) {
    return startEnd(() -> setGoal(goal), () -> setGoal(Goal.STOW));
  }

  public Command scoreCommand(boolean isLeftL1) {
    return Commands.startEnd(
        () -> {
          wantScore = true;
          leftL1 = isLeftL1;
        },
        () -> {
          wantScore = false;
          hasCoral = false;
        });
  }

  public void acceptCharacterizationInput(double input) {
    elevatorCharacterizationInput = input;
    setGoal(Goal.CHARACTERIZING);
  }

  public double getFFCharacterizationVelocity() {
    return elevator.getFFCharacterizationVelocity();
  }

  public void endCharacterizaton() {
    elevator.endCharacterizaton();
  }

  private EndEffector.Goal endEffectorGoal = EndEffector.Goal.IDLE;

  public void setEndEffectorGoal(EndEffector.Goal goal) {
    endEffectorGoal = goal;
  }

  public void setCoastOverride(BooleanSupplier override) {
    elevator.setCoastOverride(override);
  }

  public Command toggleUnjamCommand() {
    return Commands.runOnce(() -> wantUnjam = !wantUnjam);
  }

  public boolean hasCoral() {
    return hasCoral;
  }

  public void overrideCoral(boolean hasCoral) {
    this.hasCoral = hasCoral;
  }
}
