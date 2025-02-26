package frc.team4276.frc2025.subsystems.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.util.dashboard.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SlamArm extends SubsystemBase {
  public enum Goal {
    IDLE(() -> 0.0),
    OUT(new LoggedTunableNumber("SlamArm/OutCurrent", 0.0)),
    IN(new LoggedTunableNumber("SlamArm/InCurrent", 0.0)),
    CUSTOM(new LoggedTunableNumber("SlamArm/CustomCurrent", 0.0));

    private final DoubleSupplier curentSupplier;

    private Goal(DoubleSupplier currentSupplier) {
      this.curentSupplier = currentSupplier;
    }

    private double getCurrent() {
      return curentSupplier.getAsDouble();
    }
  }

  private Goal goal = Goal.IDLE;

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private BooleanSupplier coastOverride;

  private final ArmViz measuredViz;

  public SlamArm(ArmIO io) {
    this.io = io;
    io.setBrakeMode(true);

    measuredViz = new ArmViz("Measured", Color.kBlack);

    setDefaultCommand(setGoalCommand(Goal.IDLE));
  }

  public void setCoastOverride(BooleanSupplier coastOverride) {
    this.coastOverride = coastOverride;
  }

  private boolean hasFlippedCoast = false;
  private boolean wasDisabled = true;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("SlamArm", inputs);

    if (DriverStation.isDisabled()) {
      wasDisabled = true;

      io.stop();

      if (!coastOverride.getAsBoolean()) {
        hasFlippedCoast = true;
      }

      io.setBrakeMode(!coastOverride.getAsBoolean() && hasFlippedCoast);

    } else {
      if (wasDisabled) {
        io.setBrakeMode(true);
        wasDisabled = false;
      }

      hasFlippedCoast = false;

      io.runCurrent(goal.getCurrent());
      ;
    }

    measuredViz.update(inputs.positionRads);
    Logger.recordOutput("SlamArm/Goal", goal);
  }

  @AutoLogOutput
  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  @AutoLogOutput
  public Goal getGoal() {
    return goal;
  }

  public Command setGoalCommand(Goal goal) {
    return startEnd(() -> setGoal(goal), () -> setGoal(Goal.IDLE));
  }
}
