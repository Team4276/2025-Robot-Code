package frc.team4276.frc2025.subsystems.superstructure;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.subsystems.superstructure.elevator.Elevator;
import frc.team4276.frc2025.subsystems.superstructure.endeffector.EndEffector;

public class Superstructure extends SubsystemBase {
  private final Elevator elevator;
  private final EndEffector endeffector;

  private final RollerSensorsIO sensorsIO;
  private final RollerSensorsIOInputsAutoLogged sensorsInputs = new RollerSensorsIOInputsAutoLogged();

  private boolean wantScore = false;
  private boolean leftL1 = false;

  public enum Goal {
    STOW,
    INTAKE,
    L1,
    L2,
    L3,
    CHARACTERIZING
  }

  private Supplier<Goal> desiredGoal = () -> Goal.STOW;
  private Goal currentGoal = Goal.STOW;

  private Timer scoringTimer = new Timer();

  private double elevatorCharacterizationInput = 0.0;

  public Superstructure(Elevator elevator, EndEffector endeffector, RollerSensorsIO sensorsIO) {
    this.elevator = elevator;
    this.endeffector = endeffector;
    this.sensorsIO = sensorsIO;

    elevator.setCoastOverride(() -> false);

    scoringTimer.restart();
    setDefaultCommand(setGoalCommand(() -> Superstructure.Goal.STOW));
  }

  @Override
  public void periodic() {
    sensorsIO.updateInputs(sensorsInputs);
    Logger.processInputs("RollersSensors", sensorsInputs);

    if (wantScore) {
      scoringTimer.reset();
      wantScore = false;

      if (currentGoal == Goal.L1) {
        endeffector.setGoal(leftL1 ? EndEffector.Goal.FAVOR_LEFT : EndEffector.Goal.FAVOR_RIGHT);

      } else {
        endeffector.setGoal(EndEffector.Goal.SCORE);

      }
    } else if (scoringTimer.get() > 1.0) {
      endeffector.setGoal(EndEffector.Goal.IDLE);
    }

    currentGoal = desiredGoal.get();

    switch (currentGoal) {
      case STOW:
        elevator.setGoal(Elevator.Goal.STOW);
        endeffector.setGoal(EndEffector.Goal.IDLE);

        break;

      case INTAKE:
        elevator.setGoal(Elevator.Goal.STOW);
        endeffector.setGoal(EndEffector.Goal.INTAKE);

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

      case CHARACTERIZING:
        elevator.runCharacterization(elevatorCharacterizationInput);
        endeffector.setGoal(endEffectorGoal);

      default:
        break;
    }

    elevator.periodic();
    endeffector.periodic();

    SmartDashboard.putString("Comp/Superstructure/DesiredGoal", desiredGoal.get().toString());
    SmartDashboard.putString("Comp/Superstructure/CurrentGoal", currentGoal.toString());

    Logger.recordOutput("Superstructure/DesiredGoal", desiredGoal.get());
    Logger.recordOutput("Superstructure/CurrentGoal", currentGoal);
  }

  public void setGoal(Goal goal) {
    desiredGoal = () -> goal;
  }

  public void setGoal(Supplier<Goal> goal) {
    desiredGoal = goal;
  }

  public Command setGoalCommand(Goal goal) {
    return setGoalCommand(() -> goal);
  }

  public Command setGoalCommand(Supplier<Goal> goal) {
    return Commands.startEnd(() -> setGoal(goal), () -> setGoal(Goal.STOW), this);
  }

  public Goal getGoal() {
    return currentGoal;
  }

  public Command scoreCommand() {
    return scoreCommand(false);
  }

  public Command scoreCommand(boolean isLeftL1) {
    return Commands.runOnce(() -> {
      wantScore = true;
      leftL1 = isLeftL1;
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
}
