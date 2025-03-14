package frc.team4276.frc2025.subsystems.algaefier.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.frc2025.Constants;
import frc.team4276.util.dashboard.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Arm {
  public enum Goal {
    STOW(new LoggedTunableNumber("Algaefier/Arm/StowDegrees", 90.0)),
    INTAKE(new LoggedTunableNumber("Algaefier/Arm/IntakeDegrees", 90.0)),
    HOLD(new LoggedTunableNumber("Algaefier/Arm/HoldDegrees", 90.0)),
    SCORE(new LoggedTunableNumber("Algaefier/Arm/ScoreDegrees", 90.0)),
    CHARACTERIZING(() -> 90.0),
    CUSTOM(new LoggedTunableNumber("Algaefier/Arm/CustomSetpoint", 90.0));

    private final DoubleSupplier armSetpointSupplier;

    private Goal(DoubleSupplier armSetpointSupplier) {
      this.armSetpointSupplier = armSetpointSupplier;
    }

    private double getRads() {
      return Units.degreesToRadians(armSetpointSupplier.getAsDouble());
    }

    private double getDegs() {
      return armSetpointSupplier.getAsDouble();
    }
  }

  private Goal goal = Goal.STOW;

  private final LoggedTunableNumber maxVel =
      new LoggedTunableNumber("Algaefier/Arm/maxVelDeg", 0.0);
  private final LoggedTunableNumber maxAccel =
      new LoggedTunableNumber("Algaefier/Arm/maxAccelDeg", 0.0);

  private final LoggedTunableNumber kS = new LoggedTunableNumber("Algaefier/Arm/kS", 0.0);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Algaefier/Arm/kV", 0.0);
  private final LoggedTunableNumber kG = new LoggedTunableNumber("Algaefier/Arm/kG", 0.0);

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private ArmFeedforward ff =
      new ArmFeedforward(kS.getAsDouble(), kG.getAsDouble(), kV.getAsDouble(), 0.0);
  private TrapezoidProfile profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              Units.degreesToRadians(maxVel.getAsDouble()),
              Units.degreesToRadians(maxAccel.getAsDouble())));
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

  private BooleanSupplier coastOverride;

  private double characterizationInput = 0.0;

  private final ArmViz goalViz;
  private final ArmViz measuredViz;

  public Arm(ArmIO io) {
    this.io = io;
    io.setBrakeMode(true);

    goalViz = new ArmViz("Goal", Color.kGreen);
    measuredViz = new ArmViz("Measured", Color.kBlack);
  }

  public void setCoastOverride(BooleanSupplier coastOverride) {
    this.coastOverride = coastOverride;
  }

  private boolean hasFlippedCoast = false;
  private boolean wasDisabled = true;

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Algaefier/Arm", inputs);

    if (DriverStation.isDisabled()) {
      wasDisabled = true;

      io.stop();

      if (!coastOverride.getAsBoolean()) {
        hasFlippedCoast = true;
      }

      io.setBrakeMode(!(coastOverride.getAsBoolean() && hasFlippedCoast));

      setpointState = new TrapezoidProfile.State(inputs.positionRads, 0.0);

      if (Constants.isTuning) {
        ff = new ArmFeedforward(kS.getAsDouble(), kG.getAsDouble(), kV.getAsDouble(), 0.0);
        profile =
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    Units.degreesToRadians(maxVel.getAsDouble()),
                    Units.degreesToRadians(maxAccel.getAsDouble())));
      }

    } else {
      if (wasDisabled) {
        io.setBrakeMode(true);
        hasFlippedCoast = false;
        wasDisabled = false;
      }

      if (goal == Goal.CHARACTERIZING) {
        io.runVolts(characterizationInput + (kG.getAsDouble() * Math.cos(inputs.positionRads)));
      } else {
        setpointState =
            profile.calculate(0.02, setpointState, new TrapezoidProfile.State(goal.getRads(), 0.0));
        io.runSetpoint(
            setpointState.position, ff.calculate(setpointState.position, setpointState.velocity));

        Logger.recordOutput("Algaefier/Arm/SetpointState/Pos", setpointState.position);
        Logger.recordOutput("Algaefier/Arm/SetpointState/Vel", setpointState.velocity);
        Logger.recordOutput("Algaefier/Arm/GoalAngle", goal.getDegs());
      }
    }

    goalViz.update(goal.getRads());
    measuredViz.update(Constants.isSim ? goal.getRads() : inputs.positionRads);
    Logger.recordOutput("Algaefier/Arm/Goal", goal);
    Logger.recordOutput(
        "Algaefier/Arm/Measured/PositionDeg", Units.radiansToDegrees(inputs.positionRads));
    Logger.recordOutput("Algaefier/Arm/Measured/PositionRad", inputs.positionRads);
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  public Goal getGoal() {
    return goal;
  }

  public Command setGoalCommand(Goal goal) {
    return Commands.startEnd(() -> setGoal(goal), () -> setGoal(Goal.STOW));
  }

  public void runCharacterization(double output) {
    characterizationInput = output;
    goal = Goal.CHARACTERIZING;
  }

  public double getFFCharacterizationVelocity() {
    return inputs.velocityRadsPerSec;
  }

  public void endCharacterizaton() {
    characterizationInput = 0.0;
  }
}
