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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name             The configured name of the camera.
   * @param rotationSupplier The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new ArrayList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Add pose observation
      if (result.multitagResult.isPresent()) {
        var multitagResult = result.multitagResult.get();

        // Calculate robot pose
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Calculate average tag distance
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        // Add tag IDs
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        // Add observation
        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                multitagResult.fiducialIDsUsed.size(), // Tag count
                robotPose , // 3D pose estimate
                robotPose , // 3D pose estimate
                multitagResult.estimatedPose.ambiguity, // Ambiguity
                totalTagDistance / result.targets.size(), // Average tag distance
                totalTagDistance / result.targets.size(), // Average tag distance
                PoseObservationType.PHOTONVISION)); // Observation type
      } else if (result.hasTargets()) { // Single tag result
        var target = result.targets.get(0);

        // Calculate robot pose
        var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
        if (tagPose.isPresent()) {
          Transform3d fieldToTarget1 = new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
          Transform3d cameraToTarget1 = target.bestCameraToTarget;
          Transform3d fieldToCamera1 = fieldToTarget1.plus(cameraToTarget1.inverse());
          Transform3d fieldToRobot1 = fieldToCamera1.plus(robotToCamera.inverse());
          Pose3d robotPose1 = new Pose3d(fieldToRobot1.getTranslation(), fieldToRobot1.getRotation());

          Transform3d fieldToTarget2 = new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
          Transform3d cameraToTarget2 = target.altCameraToTarget;
          Transform3d fieldToCamera2 = fieldToTarget2.plus(cameraToTarget2.inverse());
          Transform3d fieldToRobot2 = fieldToCamera2.plus(robotToCamera.inverse());
          Pose3d robotPose2 = new Pose3d(fieldToRobot2.getTranslation(), fieldToRobot2.getRotation());

          // Add tag ID
          tagIds.add((short) target.fiducialId);

          // Add observation
          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(), // Timestamp
                  1, // Tag count
                  robotPose1, // 3D pose estimate
                  robotPose2, // 3D pose estimate
                  target.poseAmbiguity , // Ambiguity
                  cameraToTarget1.getTranslation().getNorm(), // Tag distances
                  cameraToTarget2.getTranslation().getNorm(), // Tag distances
                  PoseObservationType.PHOTONVISION)); // Observation type
        }
      }
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
