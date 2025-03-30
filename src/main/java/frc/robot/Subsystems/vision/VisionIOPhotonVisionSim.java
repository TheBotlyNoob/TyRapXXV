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

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
    private static VisionSystemSim visionSim;

    private final Supplier<Pose2d> poseSupplier;
    private final PhotonCameraSim cameraSim;

    /**
     * Creates a new VisionIOPhotonVisionSim.
     *
     * @param name         The name of the camera.
     * @param poseSupplier Supplier for the robot pose to use in simulation.
     */
    public VisionIOPhotonVisionSim(
            String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
        super(name, robotToCamera);
        this.poseSupplier = poseSupplier;

        // Initialize vision sim
        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(Constants.Vision.aprilTagLayout);
        }

        // Add sim camera
        var cameraProperties = new SimCameraProperties();
        // diagFov = 2*atan(sqrt(tan^2(hFov/2) + tan^2(vFov/2)))
        Angle diagFov = Units.Radians.of(2 * Math.atan(
                Math.sqrt(Math.pow(Math.tan(Math.toRadians(Constants.LimelightConstants.kHorizontalFOVDeg / 2)), 2)
                        + Math.pow(Math.tan(Math.toRadians(Constants.LimelightConstants.kVerticalFOVDeg / 2)), 2))));

        System.out.println("Diagonal FOV: " + diagFov.in(Units.Degrees) + "degrees");

        cameraProperties.setCalibration(Constants.LimelightConstants.kResolutionWidth,
                Constants.LimelightConstants.kResolutionHeight, new Rotation2d(diagFov));
        cameraSim = new PhotonCameraSim(camera, cameraProperties, Constants.Vision.aprilTagLayout);
        visionSim.addCamera(cameraSim, robotToCamera);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        visionSim.update(poseSupplier.get());
        // Logger.recordOutput("VisionSimulation/SimField", visionSim.getDebugField());
        super.updateInputs(inputs);
    }
}
