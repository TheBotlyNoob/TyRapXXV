package frc.robot.Subsystems.elevator;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorLimitsIOSim implements ElevatorLimitsIO {
    private final ElevatorSim m_sim;

    public ElevatorLimitsIOSim(ElevatorSim sim) {
        m_sim = sim;
    }

    @Override
    public void updateInputs(ElevatorLimitsIOInputs inputs) {
        // avoid the bottom/top limit switches from being very flaky
        inputs.touchingBottom = m_sim.wouldHitLowerLimit(Math.floor(m_sim.getPositionMeters() * 100.0) / 100.0 - 0.01);
        inputs.touchingTop = m_sim.wouldHitUpperLimit(Math.ceil(m_sim.getPositionMeters() * 100.0) / 100.0 + 0.01);
    }
}
