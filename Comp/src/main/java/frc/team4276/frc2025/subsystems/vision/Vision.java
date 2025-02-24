// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.team4276.frc2025.subsystems.vision;

import static frc.team4276.frc2025.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.field.FieldConstants;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private boolean[] camerasEnabled = { true, false };

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] = new Alert(
          "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        if (observation.tagCount() == 0) {
          robotPosesRejected.add(observation.pose1());
          continue;
        }

        Pose3d robotPose3d = null;
        double avgTagDist = 0.0;
        if (observation.tagCount() > 1) {
          // Add pose to log
          robotPoses.add(observation.pose1());
          robotPose3d = observation.pose1();

          avgTagDist = observation.avgTagDistance1();
        } else if (observation.tagCount() == 1) {
          // Add pose to log
          robotPoses.add(observation.pose1());
          robotPoses.add(observation.pose2());

          if (observation.ambiguity() < maxAmbiguity) {
            Rotation2d currentRotation = RobotState.getInstance().getEstimatedPose().getRotation();
            Rotation2d visionRotation0 = observation.pose1().toPose2d().getRotation();
            Rotation2d visionRotation1 = observation.pose2().toPose2d().getRotation();
            if (Math.abs(currentRotation.minus(visionRotation0).getRadians()) < Math
                .abs(currentRotation.minus(visionRotation1).getRadians())) {
              robotPose3d = observation.pose1();
              avgTagDist = observation.avgTagDistance1();
            } else {
              robotPose3d = observation.pose2();
              avgTagDist = observation.avgTagDistance2();
            }
          }

          if (robotPose3d == null) {
            continue;
          }

          // if (avgTagDist > maxDist && observation.tagCount() == 1) {
          // robotPosesRejected.add(robotPose3d);
          // continue;
          // }

          if (!camerasEnabled[cameraIndex]) {
            robotPosesRejected.add(robotPose3d);
            continue;
          }

          // Exit if robot pose is off the field
          if (robotPose3d.getX() < -fieldBorderMargin
              || robotPose3d.getX() > FieldConstants.fieldLength + fieldBorderMargin
              || robotPose3d.getY() < -fieldBorderMargin
              || robotPose3d.getY() > FieldConstants.fieldWidth + fieldBorderMargin
              || robotPose3d.getZ() < -maxZError
              || robotPose3d.getZ() > maxZError) {
            robotPosesRejected.add(robotPose3d);
            continue;
          }

          robotPosesAccepted.add(robotPose3d);

          // Calculate standard deviations
          double stdDevFactor = Math.pow(avgTagDist, 2.0) / observation.tagCount();
          double linearStdDev = linearStdDevBaseline * stdDevFactor;
          double angularStdDev = angularStdDevBaseline * stdDevFactor;
          if (cameraIndex < cameraStdDevFactors.length) {
            linearStdDev *= cameraStdDevFactors[cameraIndex];
            angularStdDev *= cameraStdDevFactors[cameraIndex];
          }

          // Send vision observation
          consumer.accept(
              robotPose3d.toPose2d(),
              observation.timestamp(),
              VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
        }

        // Log camera datadata
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
            tagPoses.toArray(new Pose3d[tagPoses.size()]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
            robotPoses.toArray(new Pose3d[robotPoses.size()]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
            robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
            robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/Enabled", camerasEnabled[cameraIndex]);
        allTagPoses.addAll(tagPoses);
        allRobotPoses.addAll(robotPoses);
        allRobotPosesAccepted.addAll(robotPosesAccepted);
        allRobotPosesRejected.addAll(robotPosesRejected);
      }

      // Log summary data
      Logger.recordOutput(
          "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
      Logger.recordOutput(
          "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
      Logger.recordOutput(
          "Vision/Summary/RobotPosesAccepted",
          allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Summary/RobotPosesRejected",
          allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  public void setEnableCamera(int cameraIndex, boolean enable) {
    camerasEnabled[cameraIndex] = enable;
  }
}
