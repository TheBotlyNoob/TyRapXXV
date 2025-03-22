package frc.robot.Subsystems.coral;

import org.ironmaple.simulation.IntakeSimulation;

import edu.wpi.first.wpilibj.DigitalInput;

public class CoralDetectionIOSim implements CoralDetectionIO {
    private final IntakeSimulation intakeSim;

    public CoralDetectionIOSim(IntakeSimulation intakeSim) {
        this.intakeSim = intakeSim;
    }

    @Override
    public void updateInputs(CoralDetectionIOInputs inputs) {
        // our IR sensor is TRUE when unblocked, and FALSE when blocked
        inputs.hasCoral = intakeSim.getGamePiecesAmount() > 0;
    }
}
