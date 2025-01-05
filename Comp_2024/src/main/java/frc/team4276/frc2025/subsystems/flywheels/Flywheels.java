package frc.team4276.frc2025.subsystems.flywheels;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.team4276.frc2025.subsystems.flywheels.FlywheelConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team4276.frc2025.RobotState;
import java.util.function.DoubleSupplier;

// TODO: need to add proper logging

public class Flywheels extends SubsystemBase {
  private FlywheelIO io;
  private FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();
  private SimpleMotorFeedforward topFF = new SimpleMotorFeedforward(ksTop, kvTop, 0.0);
  private SimpleMotorFeedforward botFF = new SimpleMotorFeedforward(ksBot, kvBot, 0.0);

  private Goal goal = Goal.IDLE;
  private boolean closedLoop;
  private SysIdRoutine m_sysIdRoutine;

  public enum Goal {
    IDLE(() -> 0.0, () -> 0.0),
    SPEAKER(RobotState.getInstance().getSpeakerAimingParameters()::flywheelRpm),
    FERRY(RobotState.getInstance().getFerryAimingParameters()::flywheelRpm),
    AMP(() -> ampTopRPM, () -> ampBotRPM),
    PREP(() -> spinUpRPM),
    SUB(() -> normalShotRPM),
    PODIUM(() -> normalShotRPM),
    BLIND_FERRY(() -> ferryRPM),
    POOP(() -> poopTopRPM, () -> poopBotRPM),
    EXHAUST(() -> exhaustRPM),
    CHARACTERIZING(() -> 0.0, () -> 0.0);
    public DoubleSupplier topRpm, bottomRpm;

    public double getTopRpm() {
      return topRpm.getAsDouble();
    }

    public double getBottomRpm() {
      return bottomRpm.getAsDouble();
    }

    Goal(DoubleSupplier topRpmSupplier, DoubleSupplier bottomRpmSupplier) {
      this.topRpm = topRpmSupplier;
      this.bottomRpm = bottomRpmSupplier;
    }

    Goal(DoubleSupplier rpmSupplier) {
      this.topRpm = rpmSupplier;
    }
  }

  public Flywheels(FlywheelIO io) {
    this.io = io;
    setDefaultCommand(runOnce(() -> setGoal(Goal.IDLE)).withName("Flywheels Idle"));

    m_sysIdRoutine =
        new SysIdRoutine(
            // Config for SysId adjust ramp rate or step voltage if necessary
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                voltage -> runCharacterization(voltage.magnitude()),
                log -> {
                  log.motor("flywheel-top")
                      // WHY CANT THEY JUST TAKE A NUMBER
                      .voltage(Volts.of(inputs.topAppliedVolts))
                      .angularPosition(Radians.of(inputs.bottomPositionRads))
                      .angularVelocity(
                          RadiansPerSecond.of(
                              RotationsPerSecond.of(inputs.topVelocityRpm / 60)
                                  .in(RadiansPerSecond)));

                  log.motor("flywheel-bottom")
                      .voltage(Volts.of(inputs.bottomAppliedVolts))
                      .angularPosition(Radians.of(inputs.bottomPositionRads))
                      .angularVelocity(
                          RadiansPerSecond.of(
                              RotationsPerSecond.of(inputs.topVelocityRpm / 60)
                                  .in(RadiansPerSecond)));
                },
                this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (DriverStation.isDisabled()) {
      setGoal(Goal.IDLE);
    }
    if (closedLoop) {
      io.runVelocity(
          topFF.calculate(this.goal.getTopRpm()), botFF.calculate(this.goal.getBottomRpm()));
    } else if (goal == Goal.IDLE) {
      io.stop();
    }
  }

  private void setGoal(Goal goal) {
    if (goal == Goal.CHARACTERIZING || goal == Goal.IDLE) {
      closedLoop = false;
      this.goal = goal;
      return;
    }
    // TODO: this will run one more time then needed atm need to fix later
    closedLoop = true;
    this.goal = goal;
  }

  public void runCharacterization(double input) {
    setGoal(Goal.CHARACTERIZING);
    io.runCharacterizationTop(input);
    io.runCharacterizationBottom(input);
  }

  public boolean isTopSpunUp() {
    return MathUtil.isNear(inputs.topVelocityRpm, goal.getTopRpm(), tolerance)
        && goal.getTopRpm() > 2000;
  }

  public boolean isBottomSpunUp() {
    return MathUtil.isNear(inputs.topVelocityRpm, goal.getBottomRpm(), tolerance)
        && goal.getBottomRpm() > 2000;
  }

  public boolean atGoal() {
    return isTopSpunUp() && isBottomSpunUp();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public Command speakerShootCommand() {
    return startEnd(() -> setGoal(Goal.SPEAKER), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Speaker Shoot");
  }

  public Command ferryShootCommand() {
    return startEnd(() -> setGoal(Goal.FERRY), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Ferry Shoot");
  }
}
