package frc.team4276.frc2025.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.subsystems.superstructure.elevator.Elevator;
import frc.team4276.frc2025.subsystems.superstructure.endeffector.EndEffector;

public class Superstructure extends SubsystemBase { // TODO: test logic
  private final Elevator elevator;
  private final EndEffector endeffector;

  private final RollerSensorsIO sensorsIO;
  private final RollerSensorsIOInputsAutoLogged sensorsInputs = new RollerSensorsIOInputsAutoLogged();

  @AutoLogOutput
  private boolean wantScore = false;

  public enum Goal {
    STOW,
    INTAKE,
    L1,
    L2,
    L3,
    CHARACTERIZING
  }

  private Goal desiredGoal = Goal.STOW;
  private Goal currentGoal = Goal.STOW;

  private Timer scoringTimer = new Timer();

  private double elevatorCharacterizationInput = 0.0;

  public Superstructure(Elevator elevator, EndEffector endeffector, RollerSensorsIO sensorsIO) {
    this.elevator = elevator;
    this.endeffector = endeffector;
    this.sensorsIO = sensorsIO;

    elevator.setCoastOverride(() -> false);

    scoringTimer.restart();
    setDefaultCommand(setGoalCommand(Superstructure.Goal.STOW));
  }

  @Override
  public void periodic() {
    sensorsIO.updateInputs(sensorsInputs);
    Logger.processInputs("RollersSensors", sensorsInputs);

    if (wantScore) {
      scoringTimer.reset();
      wantScore = false;

      endeffector.setGoal(EndEffector.Goal.SCORE);
    } else if (scoringTimer.get() > 1.0) {
      endeffector.setGoal(EndEffector.Goal.IDLE);
    }

    currentGoal = desiredGoal;

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
        elevator.requestHome();

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

      default:
        break;
    }

    elevator.periodic();
    endeffector.periodic();

    SmartDashboard.putString("Comp/Superstructure/DesiredGoal", desiredGoal.toString());
    SmartDashboard.putString("Comp/Superstructure/CurrentGoal", currentGoal.toString());

    Logger.recordOutput("Superstructure/DesiredGoal", desiredGoal);
    Logger.recordOutput("Superstructure/CurrentGoal", currentGoal);
  }

  public void setGoal(Goal goal) {
    desiredGoal = goal;
  }

  public Command setGoalCommand(Goal goal) {
    return Commands.startEnd(() -> setGoal(goal), () -> setGoal(Goal.STOW), this);
  }

  public Goal getGoal() {
    return currentGoal;
  }

  public Command scoreCommand() {
    return Commands.runOnce(() -> wantScore = true);
  }

  public void acceptCharacterizationInput(double input) {
    elevatorCharacterizationInput = input;
  }

  public double getFFCharacterizationVelocity() {
    return elevator.getFFCharacterizationVelocity();
  }

  public void endCharacterizaton() {
    elevator.endCharacterizaton();
  }
}
