package frc.robot.Subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.robot.Subsystems.vision.VisionIO.TargetObservation;

public class VisionIOConstants {
    public static final TargetObservation invalidObservation = new TargetObservation(false, -1, new Rotation2d(),
            new Rotation2d(), 0.0, 0.0, 0.0);
}
