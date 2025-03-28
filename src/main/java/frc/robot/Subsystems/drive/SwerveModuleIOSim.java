package frc.robot.Subsystems.drive;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private final SwerveModuleSimulation m_simulator;

    private final GenericMotorController m_driveMotor;
    private Voltage driveVolts = Units.Volts.of(0.0);

    private final GenericMotorController m_turningMotor;
    private Voltage turnVolts = Units.Volts.of(0.0);

    public SwerveModuleIOSim(SwerveModuleSimulation module) {
        m_simulator = module;

        m_driveMotor = m_simulator.useGenericMotorControllerForDrive().withCurrentLimit(Units.Amps.of(60));
        m_turningMotor = m_simulator.useGenericControllerForSteer().withCurrentLimit(Units.Amps.of(60));
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.driveMotorVelocity = Units.MetersPerSecond.of(m_simulator.getDriveEncoderUnGearedSpeed().in(Units.RPM)
                * Constants.Modules.kDriveEncoderRPM2MeterPerSec);
        inputs.driveMotorAngularVelocity = m_simulator.getDriveEncoderUnGearedSpeed();
        inputs.drivingMotorDistance = Units.Meters.of(m_simulator.getDriveEncoderUnGearedPosition().in(Units.Rotations)
                * Constants.Modules.kDriveEncoderRot2Meter);
        inputs.driveMotorAppliedVoltage = driveVolts;
        inputs.drivingMotorPosition = m_simulator.getDriveEncoderUnGearedPosition();
        inputs.turningMotorPosition = m_simulator.getSteerAbsoluteAngle();
        inputs.turningMotorAngularVelocity = m_simulator.getSteerAbsoluteEncoderSpeed();
        inputs.turningMotorAppliedVoltage = turnVolts;
    }

    @Override
    public void setDriveVoltage(Voltage voltage) {
        if (DriverStation.isEnabled()) {
            driveVolts = voltage;
            m_driveMotor.requestVoltage(voltage);
        } else {
            m_driveMotor.requestVoltage(Units.Volts.of(0.0));
        }
    }

    @Override
    public void setTurnVoltage(Voltage voltage) {
        if (DriverStation.isEnabled()) {
            turnVolts = voltage;
            m_turningMotor.requestVoltage(voltage);
        } else {
            m_turningMotor.requestVoltage(Units.Volts.of(0.0));
        }
    }
}
