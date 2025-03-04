package frc.team4276.frc2025;

import static frc.team4276.frc2025.subsystems.drive.DriveConstants.kinematics;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team4276.frc2025.subsystems.vision.VisionIO.TargetObservation;
import frc.team4276.util.dashboard.ElasticUI;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private Rotation2d lastGyroAngle = Rotation2d.kZero;

  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, lastGyroAngle, lastWheelPositions, Pose2d.kZero);
  private SwerveDrivePoseEstimator poseEstimatorOdom =
      new SwerveDrivePoseEstimator(kinematics, lastGyroAngle, lastWheelPositions, Pose2d.kZero);
  private TimeInterpolatableBuffer<Pose2d> odomPoseBuffer =
      TimeInterpolatableBuffer.createBuffer(2.0);

  private final Map<Integer, TxTyPoseRecord> txTyPoses = new HashMap<>();

  private ChassisSpeeds robotVelocity = new ChassisSpeeds();

  private Pose2d trajectorySetpoint = Pose2d.kZero;

  private static RobotState mInstance;

  private boolean enableSimTrajPoseEstimation = true;

  public static RobotState getInstance() {
    if (mInstance == null) {
      mInstance = new RobotState();
    }
    return mInstance;
  }

  private RobotState() {
    ElasticUI.putPoseEstimate(() -> getEstimatedPose());
  }

  /** Resets the current odometry pose. */
  public void resetPose(Pose2d pose) {
    poseEstimatorOdom.resetPose(pose);
    poseEstimator.resetPose(pose);
  }

  public void setTrajectorySetpoint(Pose2d setpoint) {
    trajectorySetpoint = setpoint;
  }

  public void addDriveSpeeds(ChassisSpeeds speeds) {
    robotVelocity = speeds;
  }

  public void addOdometryObservation(
      double timestamp, Rotation2d yaw, SwerveModulePosition[] wheelPositions) {
    // Update gyro angle
    if (yaw == null) {
      // Derive from kinematics
      yaw =
          lastGyroAngle.rotateBy(
              new Rotation2d(kinematics.toTwist2d(lastWheelPositions, wheelPositions).dtheta));
      lastGyroAngle = yaw;
    }

    lastWheelPositions = wheelPositions;

    poseEstimatorOdom.updateWithTime(timestamp, yaw, wheelPositions);
    poseEstimator.updateWithTime(timestamp, yaw, wheelPositions);
    odomPoseBuffer.addSample(timestamp, poseEstimatorOdom.getEstimatedPosition());
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    var pose = new Pose2d(visionRobotPoseMeters.getTranslation(), getEstimatedPose().getRotation());
    poseEstimator.addVisionMeasurement(pose, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Adds a new timestamped vision measurement. */
  public void addTxTyObservation(TargetObservation targetObs) {

    // Use 3D distance and tag angles to find robot pose
    // Translation2d camToTagTranslation =
    //     new Pose3d(Translation3d.kZero, new Rotation3d(0, ty, -tx))
    //         .transformBy(
    //             new Transform3d(new Translation3d(observation.distance(), 0, 0),
    // Rotation3d.kZero))
    //         .getTranslation()
    //         .rotateBy(new Rotation3d(0, cameraPose.getRotation().getY(), 0))
    //         .toTranslation2d();
    // Rotation2d camToTagRotation =
    //     robotRotation.plus(
    //         cameraPose.toPose2d().getRotation().plus(camToTagTranslation.getAngle()));
    // var tagPose2d = tagPoses2d.get(observation.tagId());
    // if (tagPose2d == null) return;
    // Translation2d fieldToCameraTranslation =
    //     new Pose2d(tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
    //         .transformBy(GeomUtil.toTransform2d(camToTagTranslation.getNorm(), 0.0))
    //         .getTranslation();
    // Pose2d robotPose =
    //     new Pose2d(
    //             fieldToCameraTranslation,
    // robotRotation.plus(cameraPose.toPose2d().getRotation()))
    //         .transformBy(new Transform2d(cameraPose.toPose2d(), Pose2d.kZero));
    // // Use gyro angle at time for robot rotation
    // robotPose = new Pose2d(robotPose.getTranslation(), robotRotation);

    // // Add transform to current odometry based pose for latency correction
    // txTyPoses.put(
    //     observation.tagId(),
    //     new TxTyPoseRecord(robotPose, camToTagTranslation.getNorm(), observation.timestamp()));
  }

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return useTrajectorySetpoint() ? trajectorySetpoint : poseEstimator.getEstimatedPosition();
  }

  @AutoLogOutput(key = "RobotState/EstimatedOdomPose")
  public Pose2d getEstimatedOdomPose() {
    return poseEstimatorOdom.getEstimatedPosition();
  }

  @AutoLogOutput(key = "RobotState/EstimatedVisionPose")
  public Pose2d getEstimatedVisionPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @AutoLogOutput(key = "RobotState/EstimatedTxTyVisionPose")
  public Pose2d
      getTxTyPose() { // TODO: add offset from poseEstimator (also find a way to expose it)

    return Pose2d.kZero;
  }

  private boolean useTrajectorySetpoint() {
    return enableSimTrajPoseEstimation
        ? false
        : Constants.getMode() == Constants.Mode.SIM && DriverStation.isAutonomousEnabled();
  }

  public Pose2d getTrajectorySetpoint() {
    return trajectorySetpoint;
  }

  @AutoLogOutput(key = "RobotState/FieldVelocity")
  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getEstimatedPose().getRotation());
  }

  public record TxTyPoseRecord() {}
}
