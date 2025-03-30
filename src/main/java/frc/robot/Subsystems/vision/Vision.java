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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.Set;
import java.util.stream.IntStream;

import static frc.robot.Constants.Vision.aprilTagLayout;

public class Vision extends SubsystemBase {
    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    private final List<Set<Integer>> fiducialIDFilters;
    private final List<LinearFilter> yawLinearFilters;

    public Vision(VisionConsumer consumer, Set<Integer> defaultFiducialIDFilter, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;

        // Initialize inputs and ID filters
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        this.fiducialIDFilters = new ArrayList<>(io.length);
        this.yawLinearFilters = new ArrayList<>(io.length);
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
            fiducialIDFilters.add(defaultFiducialIDFilter);
            yawLinearFilters.add(LinearFilter.movingAverage(10));
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] = new Alert(
                    "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
        }
    }

    /**
     * Returns the X (facing to the left, if you were to embody the camera)
     * distance to the best target,
     *
     * Make sure the target is valid before using this method.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Distance getTargetDistX(int cameraIndex) {
        return Units.Meters.of(inputs[cameraIndex].latestTargetObservation.dxMeters());
    }

    /**
     * Returns the Y (pointing downward, if you were to embody the camera) to the
     * best target.
     *
     * Make sure the target is valid before using this method.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Distance getTargetDistY(int cameraIndex) {
        return Units.Meters.of(inputs[cameraIndex].latestTargetObservation.dyMeters());
    }

    /**
     * Returns the Z (facing outward/forward, if you were to embody the camera)
     * distance to the best target.
     *
     * Make sure the target is valid before using this method.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Distance getTargetDistZ(int cameraIndex) {
        return Units.Meters.of(inputs[cameraIndex].latestTargetObservation.dzMeters());
    }

    /**
     * Returns the linearly filtered yaw angle (angle left/right of the camera) to
     * the best target.
     *
     * Make sure the target is valid before using this method.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetYaw(int cameraIndex) {
        return Rotation2d.fromRadians(yawLinearFilters.get(cameraIndex).lastValue());
    }

    /**
     * Returns the pitch angle (angle up/down from the camera) to the best target.
     *
     * Make sure the target is valid before using this method.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetPitch(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.pitch();
    }

    /**
     * Returns whether the camera's best target is valid.
     *
     * @returns - whether the best target is valid.
     */
    public boolean isTargetValid(int cameraIndex) {
        // we check the fiducial filter to not only prevent the filter from affecting
        // the pose estimation, but also to allow us to see the IO in advantagescope
        // with and without the filter
        return inputs[cameraIndex].latestTargetObservation.isValid()
                && fiducialIDFilters.get(cameraIndex)
                        .contains(Integer.valueOf(getFiducialID(cameraIndex)));

    }

    /**
     * Sets the IDs of the tags to track as possible best targets for the given
     * camera.
     *
     * This does not affect pose observation.
     *
     * @param cameraIndex The index of the camera to use.
     * @param tagIds      The IDs of the tags to track.
     */
    public void setFiducialIDFilter(int cameraIndex, Set<Integer> tagIds) {
        fiducialIDFilters.set(cameraIndex, tagIds);
    }

    /**
     * Gets the ID of the best target.
     *
     * Make sure the target is valid before using this method.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public int getFiducialID(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.fiducialID();
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose2d> allRobotPoses = new LinkedList<>();
        List<Pose2d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose2d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            if (inputs[cameraIndex].latestTargetObservation.isValid()) {
                yawLinearFilters.get(cameraIndex)
                        .calculate(inputs[cameraIndex].latestTargetObservation.yaw().getRadians());
            }
            // Update disconnected alert

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose2d> robotPoses = new LinkedList<>();
            List<Pose2d> robotPosesAccepted = new LinkedList<>();
            List<Pose2d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            Optional<Pose3d> targetTagPose;
            if (isTargetValid(cameraIndex)) {
                targetTagPose = aprilTagLayout.getTagPose(getFiducialID(cameraIndex));
            } else {
                targetTagPose = Optional.empty();
            }

            // Loop over pose observations
            for (var observation : inputs[cameraIndex].poseObservations) {
                // Check whether to reject pose
                boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
                        || (observation.tagCount() == 1
                                && observation.ambiguity() > 2.0) // Cannot be high ambiguity

                        // Must be within the field boundaries
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > aprilTagLayout.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > aprilTagLayout.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                // TODO: calculate standard deviations
                // double stdDevFactor =
                // Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                // double linearStdDev = linearStdDevBaseline * stdDevFactor;
                // double angularStdDev = angularStdDevBaseline * stdDevFactor;
                // if (observation.type() == PoseObservationType.MEGATAG_2) {
                // linearStdDev *= linearStdDevMegatag2Factor;
                // angularStdDev *= angularStdDevMegatag2Factor;
                // }
                // if (cameraIndex < cameraStdDevFactors.length) {
                // linearStdDev *= cameraStdDevFactors[cameraIndex];
                // angularStdDev *= cameraStdDevFactors[cameraIndex];
                // }

                // Send vision observation
                consumer.accept(
                        observation.pose(),
                        observation.timestamp(),
                        VecBuilder.fill(.7, .7, 9999999)// trust the vision data more than the odometry
                // VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
                );
            }

            // Log camera datadata
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
                    tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
                    robotPoses.toArray(new Pose2d[robotPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose2d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose2d[robotPosesRejected.size()]));
            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/TargetTagPose",
                    targetTagPose.map(pose -> new Pose3d[] { pose }).orElse(new Pose3d[0]));
            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/FiducialIDFilter",
                    fiducialIDFilters.get(cameraIndex).stream().mapToInt(Integer::intValue).toArray());

            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput(
                "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose2d[allRobotPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted",
                allRobotPosesAccepted.toArray(new Pose2d[allRobotPosesAccepted.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesRejected",
                allRobotPosesRejected.toArray(new Pose2d[allRobotPosesRejected.size()]));
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
                Pose2d visionRobotPoseMeters,
                double timestampSeconds,
                Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
