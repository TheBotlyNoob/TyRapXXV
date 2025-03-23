package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RangeSensor extends SubsystemBase {

    protected CANrange sensor;
    protected double validTime;
    protected double currentRangeM;

    public RangeSensor(int canId) {
        sensor = new CANrange(canId);

        // Configure the CANrange for basic use
        CANrangeConfiguration configs = new CANrangeConfiguration();
        configs.withFovParams(new FovParamsConfigs().withFOVCenterX(0.0)
                .withFOVCenterY(0.0)
                .withFOVRangeX(6.75)
                .withFOVRangeY(6.75));

        // Write these configs to the CANrange
        sensor.getConfigurator().apply(configs);
    }

    public double getRange() {
        return this.currentRangeM;
    }

    @Override
    public void periodic() {
        this.currentRangeM = sensor.getDistance(true).getValue().magnitude();
        this.validTime = sensor.getMeasurementTime().getValue().magnitude();
    }

}
