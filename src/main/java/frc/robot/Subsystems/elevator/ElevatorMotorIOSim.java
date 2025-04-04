package frc.robot.Subsystems.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ElevatorMotorIOSim implements ElevatorMotorIO {
    private final ElevatorSim elevatorSim;
    private final Timer timer = new Timer();

    public ElevatorMotorIOSim(ElevatorSim elevatorSim) {
        this.elevatorSim = elevatorSim;
        timer.reset();
        timer.start();
    }

    @Override
    public void updateInputs(ElevatorMotorIOInputs inputs) {
        elevatorSim.update(timer.get());
        timer.reset();

        // TODO: what's the math here?
        inputs.leaderRelativeEncoderPosition = Units.Rotations.of(elevatorSim.getPositionMeters() * 100);
        inputs.leaderRelativeEncoderVelocity = Units.RotationsPerSecond.of(elevatorSim.getVelocityMetersPerSecond());
    }

    @Override
    public void setLeaderVoltage(double voltage) {
        elevatorSim.setInputVoltage(voltage);
    }

    @Override
    public void resetLeaderEncoderPosition() {
        elevatorSim.setState(0.0, 0.0);
    }
}
