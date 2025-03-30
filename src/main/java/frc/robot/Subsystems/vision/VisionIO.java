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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {

        public TargetObservation latestTargetObservation = VisionIOConstants.invalidObservation;
        public PoseObservation[] poseObservations = new PoseObservation[0];
        public int[] tagIds = new int[0];
    }

    /**
     * Represents the angle and distance to a simple target, not used for pose
     * estimation. Based on position of camera.
     */
    public static record TargetObservation(boolean isValid, int fiducialID, Rotation2d yaw, Rotation2d pitch,
            double dxMeters,
            double dyMeters,
            double dzMeters) {
    }

    /** Represents a robot pose sample used for pose estimation. */
    public static record PoseObservation(
            // timestamp, in LL/PhotonVision server time
            double timestamp,
            // estimated pose of the robot
            Pose2d pose,
            // the ambiguity of the pose, in meters
            double ambiguity,
            // the number of tags in the observation
            int tagCount,
            // the average distance between tags in the observation, in meters
            double averageTagDistance,
            // the type of observation used
            PoseObservationType type) {
    }

    public static enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2,
        PHOTONVISION
    }

    public default void updateInputs(VisionIOInputs inputs) {
    }
}
