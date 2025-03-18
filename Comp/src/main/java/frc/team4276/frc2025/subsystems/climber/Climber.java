package frc.team4276.frc2025.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.SimViz;
import frc.team4276.util.dashboard.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  public enum Goal {
    IDLE(() -> 0.0, () -> 0.0),
    CLIMB(
        new LoggedTunableNumber("Climber/ClimbVolts", -12.0),
        new LoggedTunableNumber("Climber/Wheels/ClimbVolts", 0.0)),
    LATCH(() -> 0.0, new LoggedTunableNumber("Climber/Wheels/LatchVolts", 12.0)),
    RAISE(new LoggedTunableNumber("Climber/ReverseVolts", 12.0), () -> 0.0);

    private final DoubleSupplier wheelVolts;
    private final DoubleSupplier whenchVolts;

    private Goal(DoubleSupplier whenchVolts, DoubleSupplier wheelVolts) {
      this.whenchVolts = whenchVolts;
      this.wheelVolts = wheelVolts;
    }

    public double getWhenchVolts() {
      return whenchVolts.getAsDouble();
    }

    public double getWheelVolts() {
      return wheelVolts.getAsDouble();
    }
  }

  private Goal goal = Goal.IDLE;
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private BooleanSupplier override;
  private boolean isClimbing = false;
  private boolean hasFlippedCoast = false;

  public Climber(ClimberIO io) {
    this.io = io;

    setDefaultCommand(setGoalCommand(Goal.IDLE));
  }

  public void setCoastOverride(BooleanSupplier override) {
    this.override = override;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    if (!override.getAsBoolean()) {
      hasFlippedCoast = true;
    }
    io.setBrakeMode(!(override.getAsBoolean() && hasFlippedCoast));

    if (isClimbing) {
      if ((inputs.position < 6) && (goal == Goal.CLIMB)) {
        goal = Goal.IDLE;
      }
      io.runWheelsAtVolts(goal.getWheelVolts());
      io.runRunWhenchAtVolts(goal.getWhenchVolts());

    } else {
      io.runWheelsAtVolts(0.0);
      io.runRunWhenchAtVolts(0.0);
    }

    SimViz.addClimberMeasuredObs(inputs.position);
    Logger.recordOutput("Climber/Goal", goal.toString());
    Logger.recordOutput("Climber/IsClimbing", isClimbing);
    Logger.recordOutput("Climber/Position", inputs.position);
  }

  public void setIsClimbing(boolean isClimbing) {
    this.isClimbing = isClimbing;
  }

  public Command isClimbingCommand() {
    return Commands.startEnd(() -> setIsClimbing(true), () -> setIsClimbing(false));
  }

  public void setClimbState(Goal goal) {
    this.goal = goal;
  }

  public Command setGoalCommand(Goal goal) {
    return startEnd(() -> setClimbState(goal), () -> setClimbState(Goal.IDLE));
  }

  public boolean isClimbing() {
    return isClimbing;
  }
}
