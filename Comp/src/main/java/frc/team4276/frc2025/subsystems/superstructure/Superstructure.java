package frc.team4276.frc2025.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.subsystems.superstructure.displacer.Displacer;
import frc.team4276.frc2025.subsystems.superstructure.elevator.Elevator;
import frc.team4276.frc2025.subsystems.superstructure.endeffector.EndEffector;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final Elevator elevator;
  private final EndEffector endeffector;
  private final Displacer displacer;

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

  private boolean cleared1 = false; // clean up this mess
  private boolean cleared2 = false;
  private boolean cleared3 = false;
  private boolean cleared4 = false;
  private boolean cleared5 = false;

  private boolean hasCoral = false;
  private boolean displace = false;

  public Superstructure(
      Elevator elevator, EndEffector endeffector, Displacer displacer, RollerSensorsIO sensorsIO) {
    this.elevator = elevator;
    this.endeffector = endeffector;
    this.displacer = displacer;
    this.sensorsIO = sensorsIO;

    elevator.setCoastOverride(() -> false);

    setDefaultCommand(setGoalCommand(() -> Superstructure.Goal.STOW));
  }

  @Override
  public void periodic() {
    sensorsIO.updateInputs(sensorsInputs);
    Logger.processInputs("RollersSensors", sensorsInputs);

    currentGoal = desiredGoal.get();

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

    if (displace) {
      displacer.setGoal(Displacer.Goal.VROOOM);
    }

    if (currentGoal != Goal.INTAKE) {
      cleared1 = false;
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
        displacer.setGoal(Displacer.Goal.MOOORV);

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
        displacer.setGoal(Displacer.Goal.IDLE);

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
        } else if (sensorsInputs.backTripped || cleared1) {
          cleared1 = true;
          endeffector.setGoal(EndEffector.Goal.SLOINTAKE);
        } else {
          endeffector.setGoal(EndEffector.Goal.INTAKE);
        }
        displacer.setGoal(Displacer.Goal.IDLE);

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
        displacer.setGoal(Displacer.Goal.VROOOM);

        break;
      case HI_ALGAE:
        elevator.setGoal(Elevator.Goal.HI_ALGAE);
        endeffector.setGoal(EndEffector.Goal.IDLE);
        displacer.setGoal(Displacer.Goal.VROOOM);

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
    displacer.periodic();

    Logger.recordOutput("Superstructure/DesiredGoal", desiredGoal.get());
    Logger.recordOutput("Superstructure/CurrentGoal", currentGoal);
    Logger.recordOutput("Superstructure/WantScore", wantScore);
    Logger.recordOutput("Superstructure/Displace", displace);
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

  public Command toggleDisplacerCommand() {
    return Commands.runOnce(() -> displace = !displace);
  }
}
