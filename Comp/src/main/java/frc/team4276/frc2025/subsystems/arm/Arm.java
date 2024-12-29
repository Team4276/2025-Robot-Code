package frc.team4276.frc2025.subsystems.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.RobotState;
import frc.team4276.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// TODO: cleanup; refactor; add viz; add characterization
public class Arm extends SubsystemBase {
  public enum Goal {
    STOW(new LoggedTunableNumber("Arm/StowDegrees", 70.0)),
    INTAKE(new LoggedTunableNumber("Arm/IntakeDegrees", 135.0)),
    SPEAKER(RobotState.getInstance().getSpeakerAimingParameters()::fourbarSetpoint),
    FERRY(RobotState.getInstance().getFerryAimingParameters()::fourbarSetpoint),
    PREP(new LoggedTunableNumber("Arm/PrepDegrees", 90.0)),
    AMP(new LoggedTunableNumber("Arm/AmpDegrees", 135.0)),
    SUB(new LoggedTunableNumber("Arm/SubDegrees", 135.0)),
    PODIUM(new LoggedTunableNumber("Arm/PodiumDegrees", 90.0)),
    BLIND_FERRY(new LoggedTunableNumber("Arm/BlindFerryDegrees", 135.0)),
    SKIM(new LoggedTunableNumber("Arm/SkimSetpoint", 120.0)),
    CLIMB(new LoggedTunableNumber("Arm/ClimbSetpoint", 50.0)),
    POOP(new LoggedTunableNumber("Arm/PoopSetpoint", 50.0)),
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

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private BooleanSupplier coastOverride;

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

    } else {
      if (wasDisabled) {
        io.setBrakeMode(true);
        wasDisabled = false;
      }

      hasFlippedCoast = false;

      io.runSetpoint(goal.getRads());
      Logger.recordOutput("Arm/GoalAngle", goal.getDegs());
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
    return Commands.startEnd(() -> setGoal(goal), () -> setGoal(Goal.STOW));
  }
}
