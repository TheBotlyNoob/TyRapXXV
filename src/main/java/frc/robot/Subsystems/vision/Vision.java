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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.vision.VisionIO.TargetObservation;

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
import java.util.stream.Stream;

import static frc.robot.Constants.Vision.aprilTagLayout;

public class Vision extends SubsystemBase {
    private static class VisionIOHandler {
        public Set<Integer> fiducialIDFilters;
        public final VisionIO io;
        public TargetObservation lastValidObservation = null;
        public Timer timeSinceValid = new Timer();
        public final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
        public final int id;

        public final LinearFilter yawFilter = LinearFilter.movingAverage(10);
        public final LinearFilter pitchFilter = LinearFilter.movingAverage(10);

        public VisionIOHandler(int id, VisionIO io, Set<Integer> defaultFiducialIDFilter) {
            this.id = id;
            this.io = io;
            this.fiducialIDFilters = defaultFiducialIDFilter;

            timeSinceValid.reset();
        }

        public void updateInputs() {
            io.updateInputs(inputs);
            Logger.processInputs("Vision/Camera" + Integer.toString(id), inputs);

            if (inputs.latestTargetObservation.isValid()
                    && fiducialIDFilters.contains(inputs.latestTargetObservation.fiducialID())) {
                lastValidObservation = inputs.latestTargetObservation;
                timeSinceValid.stop();
                timeSinceValid.reset();
            } else {
                timeSinceValid.start();
            }
        }
    }

    private final VisionConsumer consumer;
    private final VisionIOHandler[] io;
    // private final Alert[] disconnectedAlerts;

    public Vision(VisionConsumer consumer, Set<Integer> defaultFiducialIDFilter, VisionIO... io) {
        this.consumer = consumer;
        this.io = IntStream.range(0, io.length)
                .mapToObj((id) -> new VisionIOHandler(id, io[id], defaultFiducialIDFilter))
                .toArray(VisionIOHandler[]::new);

        // Initialize inputs and ID filters

        // Initialize disconnected alerts
        // this.disconnectedAlerts = new Alert[io.length];
        // for (int i = 0; i < inputs.length; i++) {
        // disconnectedAlerts[i] = new Alert(
        // "Vision camera " + Integer.toString(i) + " is disconnected.",
        // AlertType.kWarning);
        // }
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
        return Units.Meters.of(io[cameraIndex].lastValidObservation.dxMeters());
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
        return Units.Meters.of(io[cameraIndex].lastValidObservation.dyMeters());
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
        return Units.Meters.of(io[cameraIndex].lastValidObservation.dzMeters());
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
        return Rotation2d.fromDegrees(io[cameraIndex].yawFilter.lastValue());
    }

    /**
     * Returns the pitch angle (angle up/down from the camera) to the best target.
     *
     * Make sure the target is valid before using this method.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetPitch(int cameraIndex) {
        return Rotation2d.fromDegrees(io[cameraIndex].pitchFilter.lastValue());
    }

    /**
     * Returns the time since the last valid target was observed, in seconds.
     * 
     * A return value of 0 means the target is currently valid.
     *
     * @returns - whether the best target is valid.
     */
    public double getTargetLastValid(int cameraIndex) {
        return io[cameraIndex].timeSinceValid.get();
    }

    /**
     * Returns whether the best target is valid.
     *
     * @returns - whether the best target is valid.
     */
    public boolean isTargetValid(int cameraIndex) {
        return getTargetLastValid(cameraIndex) == 0;
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
        io[cameraIndex].fiducialIDFilters = tagIds;
    }

    /**
     * Gets the ID of the best target.
     *
     * Make sure the target is valid before using this method.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public int getFiducialID(int cameraIndex) {
        return io[cameraIndex].lastValidObservation.fiducialID();
    }

    @Override
    public void periodic() {
        for (var io : io) {
            io.updateInputs();
        }

        // we check the fiducial filter to not only prevent the filter from affecting
        // the pose estimation, but also to allow us to see the IO in advantagescope
        // with and without the filter

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose2d> allRobotPoses = new LinkedList<>();
        List<Pose2d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose2d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            if (isTargetValid(cameraIndex)) {
                io[cameraIndex].yawFilter
                        .calculate(io[cameraIndex].inputs.latestTargetObservation.yaw().getDegrees());
                io[cameraIndex].pitchFilter
                        .calculate(io[cameraIndex].inputs.latestTargetObservation.pitch().getDegrees());
            }

            // Update disconnected alert

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose2d> robotPoses = new LinkedList<>();
            List<Pose2d> robotPosesAccepted = new LinkedList<>();
            List<Pose2d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : io[cameraIndex].inputs.tagIds) {
                var tagPose = aprilTagLayout.getTagPose(tagId);

                tagPose.ifPresent(tagPoses::add);
            }

            Optional<Pose3d> targetTagPose;
            if (isTargetValid(cameraIndex)) {
                targetTagPose = aprilTagLayout.getTagPose(getFiducialID(cameraIndex));
            } else {
                targetTagPose = Optional.empty();
            }

            // Loop over pose observations
            for (var observation : io[cameraIndex].inputs.poseObservations) {
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
                    io[cameraIndex].fiducialIDFilters.stream().mapToInt(Integer::intValue).sorted().toArray());

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
