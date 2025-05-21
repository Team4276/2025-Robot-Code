package frc.team4276.frc2025.subsystems.superstructure.displacer;

import frc.team4276.frc2025.subsystems.algaefier.roller.RollerIO;
import frc.team4276.frc2025.subsystems.algaefier.roller.RollerIOInputsAutoLogged;
import frc.team4276.util.dashboard.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Displacer {
  public enum Goal {
    IDLE(() -> 0.0),
    MOOORV(new LoggedTunableNumber("Displacer/MOOORVVolts", 2.0)),
    VROOOM(new LoggedTunableNumber("Displacer/VROOMVolts", -12.0));

    private final DoubleSupplier voltageGoal;

    private Goal(DoubleSupplier voltageGoal) {
      this.voltageGoal = voltageGoal;
    }

    private double getVolts() {
      return voltageGoal.getAsDouble();
    }
  }

  private Goal goal = Goal.IDLE;

  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  public Displacer(RollerIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Displacer", inputs);

    io.runVolts(goal.getVolts());

    Logger.recordOutput("Displacer/Goal", goal);
    Logger.recordOutput("Displacer/GoalVolts", goal);
  }

  public Goal getGoal() {
    return goal;
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }
}
