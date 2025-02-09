package frc.team4276.util.dashboard;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4276.util.AllianceFlipUtil;

public class ElasticUI {
  private ElasticUI() {
  }

  public static void putSwerveDrive(Supplier<SwerveModuleState[]> state, Supplier<Rotation2d> angle) {
    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> state.get()[0].angle.getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> state.get()[0].speedMetersPerSecond, null);

        builder.addDoubleProperty("Front Right Angle", () -> state.get()[1].angle.getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> state.get()[1].speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Left Angle", () -> state.get()[2].angle.getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> state.get()[2].speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Right Angle", () -> state.get()[3].angle.getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> state.get()[3].speedMetersPerSecond, null);

        builder.addDoubleProperty("Robot Angle", () -> AllianceFlipUtil.apply(angle.get()).getRadians(), null);
      }
    });
  }

  public static void putPoseEstimate(Supplier<Pose2d> estimatedPose) {
    SmartDashboard.putData("Field", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Field");

        builder.addDoubleArrayProperty("Robot", () -> new double[] { estimatedPose.get().getX()
          //  - (fieldLength / 2)
           ,
            estimatedPose.get().getY()
            //  - (fieldWidth / 2)
            ,
            estimatedPose.get().getRotation().getDegrees()
        }, null);
      }
    });
  }
}
