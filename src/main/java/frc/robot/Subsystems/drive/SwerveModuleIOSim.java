package frc.robot.Subsystems.drive;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private final SwerveModuleSimulation m_simulator;

    private final GenericMotorController m_driveMotor;
    private final GenericMotorController m_turningMotor;

    public SwerveModuleIOSim(SwerveModuleSimulation module) {
        m_simulator = module;

        m_driveMotor = m_simulator.useGenericMotorControllerForDrive().withCurrentLimit(Units.Amps.of(20));
        m_turningMotor = m_simulator.useGenericControllerForSteer().withCurrentLimit(Units.Amps.of(20));
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.driveMotorVelocity = Units.MetersPerSecond.of(m_simulator.getDriveWheelFinalSpeed().in(Units.RPM)
                * Constants.Modules.kDriveEncoderRPM2MeterPerSec);
        inputs.driveMotorAngularVelocity = m_simulator.getDriveWheelFinalSpeed();
        inputs.drivingMotorDistance = Units.Meters.of(m_simulator.getDriveWheelFinalPosition().in(Units.Rotations)
                * Constants.Modules.kDriveEncoderRot2Meter);
        inputs.drivingMotorPosition = m_simulator.getDriveWheelFinalPosition();
        inputs.turningMotorPosition = m_simulator.getSteerAbsoluteAngle();
        inputs.turningMotorAngularVelocity = m_simulator.getSteerAbsoluteEncoderSpeed();
    }

    @Override
    public void setDriveVoltage(Voltage voltage) {
        m_driveMotor.requestVoltage(voltage);
    }

    @Override
    public void setTurnVoltage(Voltage voltage) {
        m_turningMotor.requestVoltage(voltage);
    }
}
