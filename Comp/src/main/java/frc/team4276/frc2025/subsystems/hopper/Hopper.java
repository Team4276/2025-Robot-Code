package frc.team4276.frc2025.subsystems.hopper;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.util.dashboard.LoggedTunableNumber;
import frc.team4276.util.dashboard.LoggedTunableProfile;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  public enum Goal {
    IDLE(() -> 0.0),
    CLIMB(new LoggedTunableNumber("Hopper/ClimbPosition", 0.0));

    private final DoubleSupplier setpointSupplier;

    private Goal(DoubleSupplier setpointSupplier) {
      this.setpointSupplier = setpointSupplier;
    }

    private double getPosition() {
      return setpointSupplier.getAsDouble();
    }
  }

  private Goal goal = Goal.IDLE;
  private double leftOffset = 0.0;
  private double rightOffset = 0.0;

  private final HopperIO leftIo;
  private final HopperIO rightIo;

  private final HopperIOInputsAutoLogged leftInputs = new HopperIOInputsAutoLogged();
  private final HopperIOInputsAutoLogged rightInputs = new HopperIOInputsAutoLogged();

  private final LoggedTunableProfile profile = new LoggedTunableProfile("Hopper", 0.0, 0.0);
  private TrapezoidProfile.State prevLeftState = new TrapezoidProfile.State();
  private TrapezoidProfile.State prevRightState = new TrapezoidProfile.State();

  public Hopper(HopperIO leftIo, HopperIO rightIo) {
    this.leftIo = leftIo;
    this.rightIo = rightIo;

    setDefaultCommand(setGoalCommand(Goal.IDLE));
  }

  @Override
  public void periodic() {
    leftIo.updateInputs(leftInputs);
    rightIo.updateInputs(rightInputs);
    Logger.processInputs("Hopper/Left", leftInputs);
    Logger.processInputs("Hopper/Right", rightInputs);

    if (DriverStation.isDisabled()) {
      goal = Goal.IDLE;

      prevLeftState = new TrapezoidProfile.State(leftInputs.position, 0.0);
      prevRightState = new TrapezoidProfile.State(rightInputs.position, 0.0);

    } else {
      prevLeftState =
          profile.calculate(
              0.02, prevLeftState, new TrapezoidProfile.State(goal.getPosition(), 0.0));
      prevRightState =
          profile.calculate(
              0.02, prevRightState, new TrapezoidProfile.State(goal.getPosition(), 0.0));

      leftIo.runSetpoint(prevLeftState.position + leftOffset);
      rightIo.runSetpoint(prevRightState.position + rightOffset);
    }

    Logger.recordOutput("Hopper/Goal", goal.toString());
    Logger.recordOutput("Hopper/Left/Offset", leftOffset);
    Logger.recordOutput("Hopper/Left/Setpoint", prevLeftState.position);
    Logger.recordOutput("Hopper/Right/Offset", rightOffset);
    Logger.recordOutput("Hopper/Right/Setpoint", prevRightState.position);
  }

  public Command setGoalCommand(Goal goal) {
    return startEnd(() -> this.goal = goal, () -> this.goal = Goal.IDLE);
  }

  public Command zeroCommand() {
    return Commands.runOnce(
        () -> {
          leftOffset = leftInputs.position;
          rightOffset = rightInputs.position;
        });
  }
}
