package frc.team4276.frc2025.subsystems.algaefier.arm;

import static frc.team4276.frc2025.subsystems.algaefier.arm.ArmConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ArmViz {
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d arm;
  private final String key;

  public ArmViz(String key, Color color) {
    this.key = key;
    mechanism = new LoggedMechanism2d(3.0, 3.0, new Color8Bit(Color.kWhite));

    LoggedMechanismRoot2d motorRoot = mechanism.getRoot("motorPivot", 1.0, 0.4);
    arm = new LoggedMechanismLigament2d("arm", length, 0.0, 6, new Color8Bit(color));
    motorRoot.append(arm);
  }

  /** Update arm visualizer with current arm angle */
  public void update(double angleRads) {
    // Log 3D poses
    Pose3d motor =
        new Pose3d(
            new Translation3d(origin.getX(), 0, origin.getY()),
            new Rotation3d(0.0, angleRads + Math.PI, 0.0));
    Logger.recordOutput("Arm/Mechanism3d/" + key, motor);
  }
}
