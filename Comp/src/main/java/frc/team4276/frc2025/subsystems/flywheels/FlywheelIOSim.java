package frc.team4276.frc2025.subsystems.flywheels;

import com.revrobotics.sim.SparkSimFaultManager;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import edu.wpi.first.math.system.plant.DCMotor;

public class FlywheelIOSim implements FlywheelIO { // TODO: impl
  private SparkMax sparkMax = new SparkMax(0, MotorType.kBrushless);
  private SparkSim sim = new SparkSim(sparkMax, DCMotor.getNEO(1));
  private SparkSimFaultManager manager = new SparkSimFaultManager(sparkMax);

  public FlywheelIOSim() {}

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {}
}
