package frc.team4276.frc2025;

import static frc.team4276.frc2025.subsystems.drive.DriveConstants.kinematics;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team4276.frc2025.field.FieldConstants;
import frc.team4276.util.AllianceFlipUtil;
import frc.team4276.util.dashboard.ElasticUI;

import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
  private SwerveModulePosition[] lastWheelPositions = new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
  };
  private Rotation2d lastGyroAngle = Rotation2d.kZero;

  private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, lastGyroAngle,
      lastWheelPositions, Pose2d.kZero);
  private SwerveDrivePoseEstimator poseEstimatorOdom = new SwerveDrivePoseEstimator(kinematics, lastGyroAngle,
      lastWheelPositions, Pose2d.kZero);

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

  public FieldConstants.POIs getPOIs() {
    return AllianceFlipUtil.shouldFlip() ? FieldConstants.redPOIs : FieldConstants.bluePOIs;
  }

  /** Resets the current odometry pose. */
  public void resetPose(Pose2d pose) {
    poseEstimatorOdom.resetPose(pose);
    poseEstimator.resetPose(pose);
  }

  public void setTrajectorySetpoint(Pose2d setpoint) {
    trajectorySetpoint = setpoint;
  }

  public void addDriveSpeeds(ChassisSpeeds speeds){
    robotVelocity = speeds;
  }

  public void addOdometryObservation(
      double timestamp, Rotation2d yaw, SwerveModulePosition[] wheelPositions) {
    // Update gyro angle
    if (yaw == null) {
      // Derive from kinematics
      yaw = lastGyroAngle.rotateBy(
          new Rotation2d(kinematics.toTwist2d(lastWheelPositions, wheelPositions).dtheta));
      lastGyroAngle = yaw;
    }

    lastWheelPositions = wheelPositions;

    poseEstimatorOdom.updateWithTime(timestamp, yaw, wheelPositions);
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

  private boolean useTrajectorySetpoint() {
    return enableSimTrajPoseEstimation
        ? false
        : Constants.getMode() == Constants.Mode.SIM && DriverStation.isAutonomousEnabled();
  }

  public Pose2d getTrajectorySetpoint() {
    return trajectorySetpoint;
  }

  @AutoLogOutput(key = "RobotState/FieldVelocity")
  public ChassisSpeeds getFieldVelocity(){
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getEstimatedPose().getRotation());
  }
}
