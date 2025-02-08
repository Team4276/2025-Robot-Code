package frc.team4276.frc2025.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.Constants;
import frc.team4276.util.dashboard.LoggedTunableNumber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  public enum Goal {
    STOW(new LoggedTunableNumber("Arm/StowDegrees", 80.0)),
    INTAKE(new LoggedTunableNumber("Arm/IntakeDegrees", 120.0)),
    HOLD(new LoggedTunableNumber("Arm/HoldDegrees", 80.0)),
    SCORE(new LoggedTunableNumber("Arm/ScoreDegrees", 90.0)),
    CHARACTERIZING(() -> 90.0),
    CUSTOM(new LoggedTunableNumber("Arm/CustomSetpoint", 90.0));

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

  private final LoggedTunableNumber maxVel = new LoggedTunableNumber("Arm/maxVel", Math.toRadians(40.0));
  private final LoggedTunableNumber maxAccel = new LoggedTunableNumber("Arm/maxAccel", Math.toRadians(20.0));

  private final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS", 0.0);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV", 0.0); // 0.88
  private final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG", 0.0); // 0.44
  private final LoggedTunableNumber kGLoaded = new LoggedTunableNumber("Arm/kGLoaded", 0.0); // 0.73

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private ArmFeedforward ff = new ArmFeedforward(kS.getAsDouble(), kG.getAsDouble(), kV.getAsDouble(), 0.0);
  private ArmFeedforward ffLoaded = new ArmFeedforward(kS.getAsDouble(), kGLoaded.getAsDouble(), kV.getAsDouble(), 0.0);
  private TrapezoidProfile profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(maxVel.getAsDouble(), maxAccel.getAsDouble()));
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

    setDefaultCommand(setGoalCommand(Goal.STOW));
  }

  public void setCoastOverride(BooleanSupplier coastOverride) {
    this.coastOverride = coastOverride;
  }

  private boolean hasFlippedCoast = false;
  private boolean wasDisabled = true;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    if (DriverStation.isDisabled()) {
      wasDisabled = true;

      io.stop();

      if (!coastOverride.getAsBoolean()) {
        hasFlippedCoast = true;
      }

      io.setBrakeMode(!coastOverride.getAsBoolean() && hasFlippedCoast);

      setpointState = new TrapezoidProfile.State(inputs.positionRads, 0.0);

      if (Constants.isTuning) {
        ff = new ArmFeedforward(kS.getAsDouble(), kG.getAsDouble(), kV.getAsDouble(), 0.0);
        ffLoaded = new ArmFeedforward(kS.getAsDouble(), kGLoaded.getAsDouble(), kV.getAsDouble(), 0.0);
        profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVel.getAsDouble(), maxAccel.getAsDouble()));
      }

    } else {
      if (wasDisabled) {
        io.setBrakeMode(true);
        wasDisabled = false;
      }

      hasFlippedCoast = false;

      if (goal == Goal.CHARACTERIZING) {
        io.runVolts(characterizationInput + (kG.getAsDouble() * Math.cos(inputs.positionRads)));
      } else {
        setpointState = profile.calculate(0.02, setpointState, new TrapezoidProfile.State(goal.getRads(), 0.0));
        io.runSetpoint(setpointState.position,
            goal == Goal.HOLD ? ffLoaded.calculate(setpointState.position, setpointState.velocity)
                : ff.calculate(setpointState.position, setpointState.velocity));

        Logger.recordOutput("Arm/SetpointState/Pos", setpointState.position);
        Logger.recordOutput("Arm/SetpointState/Vel", setpointState.velocity);
        Logger.recordOutput("Arm/GoalAngle", goal.getDegs());

      }
    }

    goalViz.update(goal.getRads());
    measuredViz.update(inputs.positionRads);
    Logger.recordOutput("Arm/Goal", goal);
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
    return Commands.startEnd(() -> setGoal(goal), () -> setGoal(Goal.STOW), this);
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
