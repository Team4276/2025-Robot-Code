package frc.team4276.frc2025.subsystems.superstructure.elevator;

import static frc.team4276.frc2025.subsystems.superstructure.elevator.ElevatorConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ElevatorViz {
  private final String key;
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d elevator;

  public ElevatorViz(String key, Color color) {
    this.key = key;
    mechanism = new LoggedMechanism2d(3.0, 3.0);

    LoggedMechanismRoot2d motorRoot = mechanism.getRoot("base", 1.0, 0.4);
    elevator = new LoggedMechanismLigament2d(
        "elevator", length, 90.0, 6, new Color8Bit(color));
    motorRoot.append(elevator);

  }

  /** Update arm visualizer with current arm angle */
  public void update(double position) {
    elevator.setLength(position + length);
    Logger.recordOutput("Elevator/Mechanism2d/" + key, mechanism);

    // Log 3D poses
    Pose3d motor = new Pose3d(
        new Translation3d(origin.getX(), 0, origin.getY() + position),
        new Rotation3d(0.0, 0.0, 0.0));
    Logger.recordOutput("Elevator/Mechanism3d/" + key, motor);
  }
}
