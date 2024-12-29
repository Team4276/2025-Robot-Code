package frc.team4276.frc2025;

import static frc.team4276.frc2025.subsystems.drive.DriveConstants.kinematics;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.team4276.frc2025.field.FieldConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private Rotation2d lastGyroAngle = new Rotation2d();

  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, lastGyroAngle, lastWheelPositions, new Pose2d());

  private Pose2d trajectorySetpoint = new Pose2d();

  private FieldConstants.POIs POIs = FieldConstants.bluePOIs;

  private InterpolatingDoubleTreeMap speakerFourbarAngles = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap speakerFlywheelRPMs = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap ferryFourbarAngles = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap ferryFlywheelRPMs = new InterpolatingDoubleTreeMap();

  public record AimingParameters(
      Rotation2d driveHeading, double fourbarSetpoint, double flywheelRpm, double distance) {}

  private AimingParameters latestSpeakerParams;
  private AimingParameters latestFerryParams;

  private static RobotState mInstance;

  public static RobotState getInstance() {
    if (mInstance == null) {
      mInstance = new RobotState();
    }
    return mInstance;
  }

  private RobotState() {
    speakerFourbarAngles.put(1.0, 135.0);
    speakerFourbarAngles.put(1.5, 130.0);
    speakerFourbarAngles.put(2.0, 120.0);
    speakerFourbarAngles.put(2.25, 125.0);
    speakerFourbarAngles.put(2.5, 110.0);
    // kSpeakerFourbarAngles.put(2.75, 110.0);
    speakerFourbarAngles.put(3.0, 106.5);
    // kSpeakerFourbarAngles.put(3.25, 106.5);
    speakerFourbarAngles.put(3.5, 105.0);
    // kSpeakerFourbarAngles.put(3.75, 105.0);
    speakerFourbarAngles.put(4.0, 104.0);
    // kSpeakerFourbarAngles.put(4.25, 104.0);
    speakerFourbarAngles.put(4.5, 103.0);
    // kSpeakerFourbarAngles.put(4.75, 102.0);
    speakerFourbarAngles.put(5.0, 102.0);
    speakerFourbarAngles.put(5.5, 101.0);
    speakerFourbarAngles.put(6.0, 100.0);
    speakerFourbarAngles.put(6.5, 95.0);
    speakerFourbarAngles.put(7.0, 90.0);

    speakerFlywheelRPMs.put(1.0, 3500.0);
    speakerFlywheelRPMs.put(1.5, 4000.0);
    speakerFlywheelRPMs.put(2.0, 4000.0);
    speakerFlywheelRPMs.put(2.5, 4500.0);
    speakerFlywheelRPMs.put(3.0, 5000.0);
    speakerFlywheelRPMs.put(3.5, 5000.0);
    speakerFlywheelRPMs.put(4.0, 5000.0);
    speakerFlywheelRPMs.put(4.5, 5000.0);
    speakerFlywheelRPMs.put(5.0, 5000.0);
    speakerFlywheelRPMs.put(5.5, 5000.0);
    speakerFlywheelRPMs.put(6.0, 5000.0);
    speakerFlywheelRPMs.put(6.5, 5000.0);
    speakerFlywheelRPMs.put(7.0, 5000.0);

    ferryFourbarAngles.put(5.3, 130.0);
    ferryFourbarAngles.put(7.9, 130.0);
    ferryFourbarAngles.put(9.3, 130.0);
    ferryFourbarAngles.put(12.3, 135.0);

    ferryFlywheelRPMs.put(5.3, 3500.0);
    ferryFlywheelRPMs.put(7.9, 4000.0);
    ferryFlywheelRPMs.put(9.3, 4500.0);
    ferryFlywheelRPMs.put(12.3, 5000.0);
  }

  public FieldConstants.POIs getPOIs() {
    return POIs;
  }

  public void setBlue() {
    POIs = FieldConstants.bluePOIs;
  }

  public void setRed() {
    POIs = FieldConstants.redPOIs;
  }

  /** Resets the current odometry pose. */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPose(pose);
  }

  public void setTrajectorySetpoint(Pose2d setpoint) {
    trajectorySetpoint = setpoint;
  }

  public void addOdometryObservation(
      double timestamp, Rotation2d yaw, SwerveModulePosition[] wheelPositions) {
    latestSpeakerParams = null;
    latestFerryParams = null;
    // Update gyro angle
    if (yaw == null) {
      // Derive from kinematics
      yaw =
          lastGyroAngle.rotateBy(
              new Rotation2d(kinematics.toTwist2d(wheelPositions, lastWheelPositions).dtheta));
      lastGyroAngle = yaw;
    }

    lastWheelPositions = wheelPositions;

    poseEstimator.updateWithTime(timestamp, yaw, wheelPositions);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose2d getTrajectorySetpoint() {
    return trajectorySetpoint;
  }

  public AimingParameters getSpeakerAimingParameters() {
    if (latestSpeakerParams != null) {
      return latestSpeakerParams; // return cached params if no new updates
    }

    Translation2d robot_to_target =
        getPOIs().speakerCenter.minus(getEstimatedPose().getTranslation());

    double distance = robot_to_target.getNorm();
    double armAngle = speakerFourbarAngles.get(distance);
    double flywheel_speeds = speakerFlywheelRPMs.get(distance);

    latestSpeakerParams =
        new AimingParameters(robot_to_target.getAngle(), armAngle, flywheel_speeds, distance);

    return latestSpeakerParams;
  }

  public AimingParameters getFerryAimingParameters() {
    if (latestFerryParams != null) {
      return latestFerryParams; // return cached params if no new updates
    }

    Translation2d robot_to_target =
        getPOIs().speakerCenter.minus(getEstimatedPose().getTranslation());

    double distance = robot_to_target.getNorm();
    double armAngle = ferryFourbarAngles.get(distance);
    double flywheel_speeds = ferryFlywheelRPMs.get(distance);

    latestFerryParams =
        new AimingParameters(robot_to_target.getAngle(), armAngle, flywheel_speeds, distance);

    return latestFerryParams;
  }
}
