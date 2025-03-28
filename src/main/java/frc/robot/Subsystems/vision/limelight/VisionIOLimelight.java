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

package frc.robot.Subsystems.vision.limelight;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.robot.Constants;
import frc.robot.Subsystems.vision.VisionIO;
import frc.robot.Subsystems.vision.VisionIOConstants;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.IntStream;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
    private final Supplier<Rotation2d> rotationSupplier;
    private int[] allowedFiducialIds = Constants.ID.allAprilIDs;

    /**
     * Creates a new VisionIOLimelight.
     *
     * @param name             The configured name of the Limelight.
     * @param rotationSupplier Supplier for the current estimated rotation, used for
     *                         MegaTag 2.
     */
    public VisionIOLimelight(Supplier<Rotation2d> rotationSupplier) {
        LimelightHelpers.setPipelineIndex(Constants.ID.kFrontLimelightName,
                Constants.LimelightConstants.defaultPipeline);

        LimelightHelpers.SetFiducialIDFiltersOverride(Constants.ID.kFrontLimelightName, Constants.ID.allAprilIDs);

        this.rotationSupplier = rotationSupplier;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Update orientation for MegaTag 2
        LimelightHelpers.SetRobotOrientation(Constants.ID.kFrontLimelightName, rotationSupplier.get().getDegrees(), 0,
                0, 0, 0, 0);

        // Update target observation

        // 3D transform of the primary in-view AprilTag in the coordinate system of the
        // Camera (array (6)) [tx, ty, tz, pitch, yaw, roll] (meters, degrees)
        double[] targetPose = LimelightHelpers.getTargetPose_CameraSpace(Constants.ID.kFrontLimelightName);
        int targetTag = (int) LimelightHelpers.getFiducialID(Constants.ID.kFrontLimelightName);

        if (targetPose.length < 6) {
            inputs.latestTargetObservation = VisionIOConstants.invalidObservation;
        } else if (IntStream.of(allowedFiducialIds).noneMatch(id -> id == targetTag)) {
            inputs.latestTargetObservation = VisionIOConstants.invalidObservation;
        } else {
            inputs.latestTargetObservation = new TargetObservation(
                    true,
                    targetTag,
                    // [4] = yaw (angle left/right of the camera)
                    Rotation2d.fromDegrees(targetPose[4]),
                    // [3] = pitch (angle up/down of the camera)
                    Rotation2d.fromDegrees(targetPose[3]),
                    // [0] = tx (facing to the right, if you were to embody the camera)
                    //
                    // from previous code, it seems that the X value is negative compared to the
                    // proper value.
                    targetPose[0] * -1,
                    // [1] = ty (facing down, if you were to embody the camera)
                    targetPose[1],
                    // [2] = tz (facing outward/forward, if you were to embody the camera)
                    targetPose[2]);
        }

        // Read new pose observations from NetworkTables
        // TODO: do we need to support MT1?
        LimelightHelpers.PoseEstimate botPose = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.ID.kFrontLimelightName);

        inputs.poseObservations = new PoseObservation[1];
        inputs.poseObservations[0] = new PoseObservation(
                // Timestamp, based on robot timestamp of publish and latency
                botPose.timestampSeconds,

                // 2D pose estimate
                botPose.pose,

                // Ambiguity, zeroed because the pose is already disambiguated
                0.0,

                // Tag count
                botPose.tagCount,

                // Average tag distance
                botPose.avgTagDist,

                // Observation type
                PoseObservationType.MEGATAG_2);

        Set<Integer> tagIds = new HashSet<>();
        for (LimelightHelpers.RawFiducial fiducial : botPose.rawFiducials) {
            tagIds.add(fiducial.id);
        }

        // Save tag IDs to inputs objects
        inputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            inputs.tagIds[i++] = id;
        }
    }

    @Override
    public void setFiducialIDFilter(int[] tagIds) {
        // we don't override the filter on the LIMELIGHT, so it doesn't affect
        // localization
        allowedFiducialIds = tagIds;
    }
}
