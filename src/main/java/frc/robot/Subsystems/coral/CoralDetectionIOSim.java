package frc.robot.Subsystems.coral;

import java.util.ArrayList;
import java.util.function.Supplier;
import java.util.stream.Stream;

import org.ironmaple.simulation.IntakeSimulation;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class CoralDetectionIOSim implements CoralDetectionIO {
    private final IntakeSimulation intakeSim;
    private final Supplier<Pose2d> robotPoseSupplier;

    private static final Translation2d[] humanPlayerCoords = new Translation2d[] {
            // april tag 12, player station on the blue left, looking from the barge
            new Translation2d(Units.Inches.of(33.51), Units.Inches.of(25.80)),
            // april tag 13, player station on the blue right, looking from the barge
            new Translation2d(Units.Inches.of(33.51), Units.Inches.of(291.20)),
            // april tag 2, player station on the red left, looking from the barge
            new Translation2d(Units.Inches.of(657.37), Units.Inches.of(291.20)),
            // april tag 1, player station on the red right, looking from the barge
            new Translation2d(Units.Inches.of(657.37), Units.Inches.of(25.80)),
    };

    public CoralDetectionIOSim(IntakeSimulation intakeSim, Supplier<Pose2d> robotPoseSupplier) {
        this.intakeSim = intakeSim;
        this.robotPoseSupplier = robotPoseSupplier;
    }

    @Override
    public void updateInputs(CoralDetectionIOInputs inputs) {
        Translation2d robotPos = robotPoseSupplier.get().getTranslation();
        // if we're within .25 meters of a human player station, assume we've collected
        // one
        if (Stream.of(humanPlayerCoords)
                .anyMatch(coord -> coord.getDistance(robotPos) < 0.25)) {
            intakeSim.addGamePieceToIntake();
        }
        // if we're near a coral station, just assume we've collected one
        inputs.hasCoral = intakeSim.getGamePiecesAmount() > 0;
    }
}
