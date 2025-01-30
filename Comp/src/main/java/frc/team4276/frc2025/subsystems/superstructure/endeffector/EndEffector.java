package frc.team4276.frc2025.subsystems.superstructure.endeffector;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {
  public enum Goal {
    IDLE(() -> 0.0, () -> 0.0),
    INTAKE(() -> 0.0, () -> 0.0),
    SCORE(() -> 0.0, () -> 0.0);

    private final DoubleSupplier leftVoltageGoal;
    private final DoubleSupplier rightVoltageGoal;

    private Goal(DoubleSupplier leftVoltageGoal, DoubleSupplier rightVoltageGoal) {
      this.leftVoltageGoal = leftVoltageGoal;
      this.rightVoltageGoal = rightVoltageGoal;
    }

    private double getLeftVolts() {
      return leftVoltageGoal.getAsDouble();
    }

    private double getRightVolts() {
      return rightVoltageGoal.getAsDouble();
    }

  }

  private Goal goal = Goal.IDLE;

  private final EndEffectorIO io;
  private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

  public EndEffector(EndEffectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("EndEffector", inputs);

    if (DriverStation.isDisabled()) {
      goal = Goal.IDLE;
    }

    io.runVolts(goal.getRightVolts());
    Logger.recordOutput("EndEffector/Goal", goal);
  }

  @AutoLogOutput
  public Goal getGoal() {
    return goal;
  }

  @AutoLogOutput
  public void setGoal(Goal goal) {
    this.goal = goal;
  }
}
