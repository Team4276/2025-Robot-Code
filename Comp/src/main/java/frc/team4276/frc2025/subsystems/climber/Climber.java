package frc.team4276.frc2025.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.subsystems.climber.Climber.Goal;
import frc.team4276.util.dashboard.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class Climber extends SubsystemBase {
  static DoubleSupplier hookVolts = new LoggedTunableNumber("EndEffector/HookVolts", 5.0);
  static DoubleSupplier whenchVolts = new LoggedTunableNumber("EndEffector/ClimbVolts", 5.0);
  static DoubleSupplier reverseVolts = new LoggedTunableNumber("EndEffector/ReverseVolts", -5.0);

  public enum Goal {
    IDLE(() -> 0.0, () -> 0.0),
    HOLD(() -> 0.0, () -> 0.0),
    CLIMB(whenchVolts, hookVolts),
    HOOK(() -> 0.0, hookVolts),
    REVERSE(() -> 0.0, reverseVolts);

    private final DoubleSupplier hookVolatge;
    private final DoubleSupplier whenchVoltage;

    private Goal(DoubleSupplier leftVoltageGoal, DoubleSupplier rightVoltageGoal) {
      this.whenchVoltage = leftVoltageGoal;
      this.hookVolatge = rightVoltageGoal;
    }

    private Goal(DoubleSupplier voltageGoal) {
      this.whenchVoltage = voltageGoal;
      this.hookVolatge = voltageGoal;
    }

    public double getWhenchVolts() {
      return whenchVoltage.getAsDouble();
    }

    public double gethookVolts() {
      return hookVolatge.getAsDouble();
    }
  }

  private Goal goal;
  private final ClimberIO climberIO;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private boolean isClimbing = false;

  public Climber(ClimberIO io) {
    climberIO = io;
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(inputs);
    if (isClimbing) {
      climberIO.runWheelsAtVolts(goal.gethookVolts());
      climberIO.runRunWhenchAtVolts(goal.getWhenchVolts());
    }
  }

  public void setClimbState(Goal goal) {
    this.goal = goal;
  }

  public Command hookCommand() {
    return startEnd(() -> setClimbState(Goal.HOOK), () -> setClimbState(Goal.IDLE));
  }

  public Command climbCommand() {
    return startEnd(() -> setClimbState(Goal.CLIMB), () -> setClimbState(Goal.IDLE));
  }

  public Command holdCommand() {
    return startEnd(() -> setClimbState(Goal.HOLD), () -> setClimbState(Goal.IDLE));
  }

  public Command stopCommand() {
    return startEnd(() -> setClimbState(Goal.IDLE), () -> setClimbState(Goal.IDLE));
  }
}
