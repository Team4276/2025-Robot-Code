package frc.team4276.frc2025.subsystems.superstructure.elevator;

import static frc.team4276.frc2025.subsystems.superstructure.elevator.ElevatorConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.util.Color;
import org.littletonrobotics.junction.Logger;

public class ElevatorViz {
  private final String key;

  public ElevatorViz(String key, Color color) {
    this.key = key;
  }

  /** Update arm visualizer with current arm angle */
  public void update(double position) {
    // Log 3D poses
    Pose3d motor =
        new Pose3d(
            new Translation3d(origin.getX(), position, origin.getY()),
            new Rotation3d(0.0, 0.0, 0.0));
    Logger.recordOutput("Arm/Mechanism3d/" + key, motor);
  }
}
