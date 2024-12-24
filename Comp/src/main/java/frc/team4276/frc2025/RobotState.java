package frc.team4276.frc2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.team4276.frc2025.field.Field;

public class RobotState {
  private Field.POIs mPOIs = Field.Red.kPOIs;

  private InterpolatingDoubleTreeMap kSpeakerFourbarAngles = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap kSpeakerFlywheelRPMs = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap kFerryFourbarAngles = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap kFerryFlywheelRPMs = new InterpolatingDoubleTreeMap();

  public record AimingParameters(
      Rotation2d driveHeading, double fourbarSetpoint, double flywheelRpm, double distance) {

    public Rotation2d getDriveHeading() {
      return driveHeading;
    }

    public double getFourbarSetpoint() {
      return fourbarSetpoint;
    }

    public double getFlywheelRpm() {
      return flywheelRpm;
    }

    public double getDistance() {
      return distance;
    }
  }

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
    kSpeakerFourbarAngles.put(1.0, 135.0);
    kSpeakerFourbarAngles.put(1.5, 130.0);
    kSpeakerFourbarAngles.put(2.0, 120.0);
    kSpeakerFourbarAngles.put(2.25, 125.0);
    kSpeakerFourbarAngles.put(2.5, 110.0);
    // kSpeakerFourbarAngles.put(2.75, 110.0);
    kSpeakerFourbarAngles.put(3.0, 106.5);
    // kSpeakerFourbarAngles.put(3.25, 106.5);
    kSpeakerFourbarAngles.put(3.5, 105.0);
    // kSpeakerFourbarAngles.put(3.75, 105.0);
    kSpeakerFourbarAngles.put(4.0, 104.0);
    // kSpeakerFourbarAngles.put(4.25, 104.0);
    kSpeakerFourbarAngles.put(4.5, 103.0);
    // kSpeakerFourbarAngles.put(4.75, 102.0);
    kSpeakerFourbarAngles.put(5.0, 102.0);
    kSpeakerFourbarAngles.put(5.5, 101.0);
    kSpeakerFourbarAngles.put(6.0, 100.0);
    kSpeakerFourbarAngles.put(6.5, 95.0);
    kSpeakerFourbarAngles.put(7.0, 90.0);

    kSpeakerFlywheelRPMs.put(1.0, 3500.0);
    kSpeakerFlywheelRPMs.put(1.5, 4000.0);
    kSpeakerFlywheelRPMs.put(2.0, 4000.0);
    kSpeakerFlywheelRPMs.put(2.5, 4500.0);
    kSpeakerFlywheelRPMs.put(3.0, 5000.0);
    kSpeakerFlywheelRPMs.put(3.5, 5000.0);
    kSpeakerFlywheelRPMs.put(4.0, 5000.0);
    kSpeakerFlywheelRPMs.put(4.5, 5000.0);
    kSpeakerFlywheelRPMs.put(5.0, 5000.0);
    kSpeakerFlywheelRPMs.put(5.5, 5000.0);
    kSpeakerFlywheelRPMs.put(6.0, 5000.0);
    kSpeakerFlywheelRPMs.put(6.5, 5000.0);
    kSpeakerFlywheelRPMs.put(7.0, 5000.0);

    kFerryFourbarAngles.put(5.3, 130.0);
    kFerryFourbarAngles.put(7.9, 130.0);
    kFerryFourbarAngles.put(9.3, 130.0);
    kFerryFourbarAngles.put(12.3, 135.0);

    kFerryFlywheelRPMs.put(5.3, 3500.0);
    kFerryFlywheelRPMs.put(7.9, 4000.0);
    kFerryFlywheelRPMs.put(9.3, 4500.0);
    kFerryFlywheelRPMs.put(12.3, 5000.0);
  }

  public synchronized Field.POIs getPOIs() {
    return mPOIs;
  }

  public synchronized void setBlue() {
    mPOIs = Field.Blue.kPOIs;
  }

  public synchronized void setRed() {
    mPOIs = Field.Red.kPOIs;
  }

  public AimingParameters getSpeakerAimingParameters() { // TODO: fix this
    if (latestSpeakerParams != null) {
      return latestSpeakerParams; // return cached params if no new updates
    }

    Translation2d robot_to_target = getPOIs().kSpeakerCenter.minus(new Pose2d().getTranslation());

    double distance = robot_to_target.getNorm();
    double armAngle = kSpeakerFourbarAngles.get(distance);
    double flywheel_speeds = kSpeakerFlywheelRPMs.get(distance);

    latestSpeakerParams =
        new AimingParameters(robot_to_target.getAngle(), armAngle, flywheel_speeds, distance);

    return latestSpeakerParams;
  }

  public AimingParameters getFerryAimingParameters() {
    if (latestFerryParams != null) {
      return latestFerryParams; // return cached params if no new updates
    }

    Translation2d robot_to_target = getPOIs().kSpeakerCenter.minus(new Pose2d().getTranslation());

    double distance = robot_to_target.getNorm();
    double armAngle = kFerryFourbarAngles.get(distance);
    double flywheel_speeds = kFerryFlywheelRPMs.get(distance);

    latestFerryParams =
        new AimingParameters(robot_to_target.getAngle(), armAngle, flywheel_speeds, distance);

    return latestFerryParams;
  }
}
