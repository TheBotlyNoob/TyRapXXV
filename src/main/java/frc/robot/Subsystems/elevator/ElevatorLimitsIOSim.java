package frc.robot.Subsystems.elevator;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorLimitsIOSim implements ElevatorLimitsIO {
    private final ElevatorSim m_sim;

    public ElevatorLimitsIOSim(ElevatorSim sim) {
        m_sim = sim;
    }

    @Override
    public void updateInputs(ElevatorLimitsIOInputs inputs) {
        inputs.touchingBottom = m_sim.hasHitLowerLimit();
        inputs.touchingTop = m_sim.hasHitUpperLimit();
    }
}
