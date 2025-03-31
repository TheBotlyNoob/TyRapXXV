// Copyright 2021-2025 FRC 6328
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

package frc.robot.Subsystems.vision;

//import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.stream.IntStream;

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
        // Read new camera observations
        Set<Short> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();
        for (var result : camera.getAllUnreadResults()) {
            // Update latest target observation
            if (result.hasTargets()) {
                PhotonTrackedTarget bestTarget = result.getBestTarget();
                Logger.recordOutput("VisionIOPhotonVision/Ambiguity", bestTarget.getPoseAmbiguity());
                inputs.latestTargetObservation = new TargetObservation(true,
                        bestTarget.fiducialId,
                        Rotation2d.fromDegrees(bestTarget.getYaw() * -1),
                        Rotation2d.fromDegrees(bestTarget.getPitch()),
                        // object/fiducial tag space
                        // Y = left
                        bestTarget.getBestCameraToTarget().getY(),
                        // Z = up
                        bestTarget.getBestCameraToTarget().getZ() * -1,
                        // X = forward
                        bestTarget.getBestCameraToTarget().getX());
            } else {
                inputs.latestTargetObservation = VisionIOConstants.invalidObservation;
            }

            // Add pose observation
            if (result.multitagResult.isPresent()) { // Multitag result
                var multitagResult = result.multitagResult.get();

                // Calculate robot pose
                Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                Pose2d robotPose = new Pose2d(fieldToRobot.getTranslation().toTranslation2d(),
                        fieldToRobot.getRotation().toRotation2d());

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
                                robotPose, // 3D pose estimate
                                multitagResult.estimatedPose.ambiguity, // Ambiguity
                                multitagResult.fiducialIDsUsed.size(), // Tag count
                                totalTagDistance / result.targets.size(), // Average tag distance
                                PoseObservationType.PHOTONVISION)); // Observation type

            } else if (!result.targets.isEmpty()) { // Single tag result
                var target = result.targets.get(0);

                // Calculate robot pose
                var tagPose = Constants.Vision.aprilTagLayout.getTagPose(target.fiducialId);
                if (tagPose.isPresent()) {
                    Transform3d fieldToTarget = new Transform3d(tagPose.get().getTranslation(),
                            tagPose.get().getRotation());
                    Transform3d cameraToTarget = target.getBestCameraToTarget();
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                    Pose2d robotPose = new Pose2d(fieldToRobot.getTranslation().toTranslation2d(),
                            fieldToRobot.getRotation().toRotation2d());

                    // Add tag ID
                    tagIds.add((short) target.fiducialId);

                    // Add observation
                    poseObservations.add(
                            new PoseObservation(
                                    result.getTimestampSeconds(), // Timestamp
                                    robotPose, // 3D pose estimate
                                    target.poseAmbiguity, // Ambiguity
                                    1, // Tag count
                                    cameraToTarget.getTranslation().getNorm(), // Average tag distance
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
