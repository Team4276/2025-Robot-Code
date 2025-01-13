package frc.team4276.frc2025.subsystems.superstructure.elevator;

import static frc.team4276.frc2025.subsystems.superstructure.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase { // TODO: config; tune
  public enum Goal {
    STOW(new LoggedTunableNumber("Elevator/StowPosition", 0.0)),
    L1(new LoggedTunableNumber("Elevator/L1Position", 0.0)),
    L2(new LoggedTunableNumber("Elevator/L2Position", 0.0)),
    L3(new LoggedTunableNumber("Elevator/L3Position", 0.0)),
    CHARACTERIZING(() -> 0.0),
    CUSTOM(new LoggedTunableNumber("Elevator/CustomSetpoint", 0.0));

    private final DoubleSupplier elevatorSetpointSupplier;

    private Goal(DoubleSupplier elevatorSetpointSupplier) {
      this.elevatorSetpointSupplier = elevatorSetpointSupplier;
    }

    private double getPosition() {
      return elevatorSetpointSupplier.getAsDouble();
    }
  }

  private Goal goal = Goal.STOW;

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private BooleanSupplier coastOverride;

  private double characterizationInput = 0.0;
  
  private boolean wantHome = false;
  private boolean isHoming = false;

  private final ElevatorViz goalViz;
  private final ElevatorViz measuredViz;

  public Elevator(ElevatorIO io) {
    this.io = io;
    io.setBrakeMode(true);

    goalViz = new ElevatorViz("Goal", Color.kGreen);
    measuredViz = new ElevatorViz("Measured", Color.kBlack);
  }

  public void setCoastOverride(BooleanSupplier coastOverride) {
    this.coastOverride = coastOverride;
  }

  private boolean hasFlippedCoast = false;
  private boolean wasDisabled = true;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    if(inputs.botLimit){
      io.setPosition(homePosition);
      isHoming = false;

    } else if(inputs.topLimit){
      io.setPosition(maxPosition);
      isHoming = false;

    }

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
      
      if(goal != Goal.STOW){
        isHoming = false;
      }

      if(goal == Goal.CHARACTERIZING){
        io.runVolts(characterizationInput);

      } else if (wantHome && goal == Goal.STOW && atGoal()) {
        wantHome = false;
        isHoming = true;

      } else if(isHoming && goal == Goal.STOW){
        io.runVolts(0.0);

      } else {
        io.runSetpoint(goal.getPosition());
        Logger.recordOutput("Elevator/GoalAngle", goal.getPosition());

      }
    }

    goalViz.update(goal.getPosition());
    measuredViz.update(inputs.position);
    Logger.recordOutput("Elevator/Goal", goal);
  }

  @AutoLogOutput
  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  @AutoLogOutput
  public Goal getGoal() {
    return goal;
  }

  public boolean atGoal(){
    return MathUtil.isNear(goal.getPosition(), inputs.position, tolerance); 
  }

  public void runCharacterization(double output) {
    characterizationInput = output;
  }

  public double getFFCharacterizationVelocity() {
    return inputs.velocity;
  }

  public void endCharacterizaton() {
    characterizationInput = 0.0;
  }

  public void requestHome(){
    wantHome = true;
  }
}
