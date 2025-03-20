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
    private Voltage driveVolts = Units.Volts.of(0.0);

    private final GenericMotorController m_turningMotor;
    private Voltage turnVolts = Units.Volts.of(0.0);

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
        inputs.driveMotorAppliedVoltage = driveVolts;
        inputs.drivingMotorPosition = m_simulator.getDriveWheelFinalPosition();
        inputs.turningMotorPosition = m_simulator.getSteerAbsoluteAngle();
        inputs.turningMotorAngularVelocity = m_simulator.getSteerAbsoluteEncoderSpeed();
        inputs.turningMotorAppliedVoltage = turnVolts;
    }

    @Override
    public void setDriveVoltage(Voltage voltage) {
        driveVolts = voltage;
        m_driveMotor.requestVoltage(voltage);
    }

    @Override
    public void setTurnVoltage(Voltage voltage) {
        turnVolts = voltage;
        m_turningMotor.requestVoltage(voltage);
    }
}
