package frc.team4276.frc2025.subsystems.superstructure.endeffector;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.util.dashboard.LoggedTunableNumber;

public class EndEffector extends SubsystemBase {
  private static final LoggedTunableNumber favorVolts = new LoggedTunableNumber("EndEffector/FavorVolts", 10.0);
  private static final LoggedTunableNumber lagVolts = new LoggedTunableNumber("EndEffector/LagVolts", 4.0);

  public enum Goal {
    IDLE(() -> 0.0, () -> 0.0),
    INTAKE(new LoggedTunableNumber("EndEffector/IntakeVolts", 7.0)),
    SCORE(new LoggedTunableNumber("EndEffector/ScoreVolts", 8.0)),
    FAVOR_LEFT(favorVolts, lagVolts),
    FAVOR_RIGHT(lagVolts, favorVolts);

    private final DoubleSupplier rightVoltageGoal;
    private final DoubleSupplier leftVoltageGoal;

    private Goal(DoubleSupplier leftVoltageGoal, DoubleSupplier rightVoltageGoal) {
      this.leftVoltageGoal = leftVoltageGoal;
      this.rightVoltageGoal = rightVoltageGoal;
    }

    private Goal(DoubleSupplier voltageGoal) {
      this.leftVoltageGoal = voltageGoal;
      this.rightVoltageGoal = voltageGoal;
    }

    public double getLeftVolts() {
      return leftVoltageGoal.getAsDouble();
    }

    public double getRightVolts() {
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

    io.runVolts(goal.getLeftVolts(), goal.getRightVolts());
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
