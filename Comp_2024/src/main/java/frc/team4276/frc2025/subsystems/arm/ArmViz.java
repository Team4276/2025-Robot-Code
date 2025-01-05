package frc.team4276.frc2025.subsystems.arm;

import static frc.team4276.frc2025.subsystems.arm.ArmConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.team4276.util.feedforwards.FourbarFeedForward;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ArmViz {
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d arm;
  private final LoggedMechanismLigament2d shooter;
  private final LoggedMechanismLigament2d leg;
  private final LoggedMechanismLigament2d bot;
  private final String key;
  private final FourbarFeedForward ff;

  public ArmViz(String key, Color color) {
    this.key = key;
    mechanism = new LoggedMechanism2d(3.0, 3.0, new Color8Bit(Color.kWhite));

    ff = new FourbarFeedForward(kFeedForwardConstants);

    LoggedMechanismRoot2d motorRoot = mechanism.getRoot("motorPivot", 1.0, 0.4);
    arm =
        new LoggedMechanismLigament2d(
            "arm", kFeedForwardConstants.kMotorLegLength, 0.0, 6, new Color8Bit(color));
    shooter =
        new LoggedMechanismLigament2d(
            "shooter", kFeedForwardConstants.kTopLength, 0.0, 6, new Color8Bit(color));
    leg =
        new LoggedMechanismLigament2d(
            "leg", kFeedForwardConstants.kSupportLegLength, 0.0, 6, new Color8Bit(color));
    bot =
        new LoggedMechanismLigament2d(
            "bot", kFeedForwardConstants.kBottomLength, 0.0, 6, new Color8Bit(color));
    motorRoot.append(arm);
    arm.append(shooter);
    shooter.append(leg);
    leg.append(bot);
  }

  /** Update arm visualizer with current arm angle */
  public void update(double angleRads) {
    // Log Mechanism2d
    double mech2dAngleRads = Math.toRadians(180.0) - angleRads;
    double[] positions = ff.getInsideAngles(mech2dAngleRads);
    arm.setAngle(Rotation2d.fromRadians(mech2dAngleRads));
    shooter.setAngle(Rotation2d.fromRadians(-1.0 * (Math.toRadians(180.0) - positions[0])));
    leg.setAngle(Rotation2d.fromRadians(-1.0 * (Math.toRadians(180.0) - positions[1])));
    bot.setAngle(Rotation2d.fromRadians(-1.0 * (Math.toRadians(180.0) - positions[2])));
    Logger.recordOutput("Arm/Mechanism2d/" + key, mechanism);

    // Log 3D poses
    Pose3d motor =
        new Pose3d(
            new Translation3d(armOrigin.getX(), 0, armOrigin.getY()),
            new Rotation3d(0.0, -angleRads, 0.0));
    Pose3d shooter =
        motor.transformBy(
            new Transform3d(
                new Translation3d(kFeedForwardConstants.kMotorLegLength, 0.0, 0.0),
                new Rotation3d(0.0, Math.toRadians(90.0) + positions[0], 0.0)));
    Pose3d support =
        shooter.transformBy(
            new Transform3d(
                new Translation3d(0.0, 0.0, -kFeedForwardConstants.kTopLength),
                new Rotation3d(0.0, Math.toRadians(90.0) + positions[1], 0.0)));
    Logger.recordOutput("Arm/Mechanism3d/" + key, motor, shooter, support);
  }
}
