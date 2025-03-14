package frc.team4276.frc2025.subsystems.algaefier.roller;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.util.dashboard.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Gripper {
  public enum Goal {
    IDLE(() -> 0.0),
    INTAKE(new LoggedTunableNumber("Algaefier/Gripper/IntakeVolts", 7.0)),
    HOLD(new LoggedTunableNumber("Algaefier/Gripper/HoldVolts", 0.5)),
    SCORE(new LoggedTunableNumber("Algaefier/Gripper/ScoreVolts", -10.0));

    private final DoubleSupplier voltageGoal;

    private Goal(DoubleSupplier voltageGoal) {
      this.voltageGoal = voltageGoal;
    }

    private double getVolts() {
      return voltageGoal.getAsDouble();
    }
  }

  private Goal goal = Goal.IDLE;
  private boolean hasGamePiece = false;

  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  public Gripper(RollerIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Algaefier/Gripper", inputs);

    if (DriverStation.isDisabled()) {
      goal = Goal.IDLE;
    }

    if (inputs.supplyCurrentAmps > 40.0) {
      hasGamePiece = true;
    }

    if (goal == Goal.SCORE) {
      hasGamePiece = false;
    }

    if (goal == Goal.IDLE) {
      io.runVolts(hasGamePiece ? Goal.HOLD.getVolts() : Goal.IDLE.getVolts());

    } else {
      io.runVolts(goal.getVolts());
    }

    Logger.recordOutput("Algaefier/Gripper/Goal", goal);
    Logger.recordOutput("Algaefier/Gripper/HasGamePiece", hasGamePiece);
  }

  public Goal getGoal() {
    return goal;
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  public Command setGoalCommand(Goal goal) {
    return Commands.startEnd(() -> setGoal(goal), () -> setGoal(Goal.IDLE));
  }

  public boolean hasGamePiece() {
    return hasGamePiece;
  }
}
