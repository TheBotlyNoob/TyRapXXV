package frc.robot.Subsystems.elevator;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import frc.robot.Constants;

public class ElevatorConfigIONetworkTables implements ElevatorConfigIO {
    private final DoubleEntry m_maxVel;
    private final DoubleEntry m_maxAccel;
    private final DoubleEntry m_ks;
    private final DoubleEntry m_kg;
    private final DoubleEntry m_kv;
    private final DoubleEntry m_ka;
    private final DoubleEntry m_kp;
    private final DoubleEntry m_ki;
    private final DoubleEntry m_kd;
    private final DoubleEntry m_decelProp;

    private final DoubleEntry m_level1Height;
    private final DoubleEntry m_level2Height;
    private final DoubleEntry m_level3Height;
    private final DoubleEntry m_level4Height;

    public ElevatorConfigIONetworkTables(NetworkTableInstance nt) {
        NetworkTable table = nt.getTable("Tuning").getSubTable("ElevatorSubsystemConfig");

        m_maxVel = table.getDoubleTopic("maxVelocityRotPerSec").getEntry(Constants.Elevator.kMaxVelocity);
        m_maxVel.set(m_maxVel.get());

        m_maxAccel = table.getDoubleTopic("maxAccelRotSecPerSec").getEntry(Constants.Elevator.kMaxAcceleration);
        m_maxAccel.set(m_maxAccel.get());

        m_decelProp = table.getDoubleTopic("decelProp").getEntry(Constants.Elevator.kDecelProp);
        m_decelProp.set(m_decelProp.get());

        NetworkTable feedForward = table.getSubTable("FeedForward");
        m_ks = feedForward.getDoubleTopic("ks").getEntry(Constants.Elevator.FF.kS);
        m_ks.set(m_ks.get());

        m_kg = feedForward.getDoubleTopic("kg").getEntry(Constants.Elevator.FF.kG);
        m_kg.set(m_kg.get());

        m_kv = feedForward.getDoubleTopic("kv").getEntry(Constants.Elevator.FF.kV);
        m_kv.set(m_kv.get());

        m_ka = feedForward.getDoubleTopic("ka").getEntry(Constants.Elevator.FF.kA);
        m_ka.set(m_ka.get());

        NetworkTable PID = table.getSubTable("PID");
        m_kp = PID.getDoubleTopic("kp").getEntry(Constants.Elevator.PID.kP);
        m_kp.set(m_ka.get());

        m_ki = PID.getDoubleTopic("ki").getEntry(Constants.Elevator.PID.kI);
        m_ki.set(m_ka.get());

        m_kd = PID.getDoubleTopic("kd").getEntry(Constants.Elevator.PID.kD);
        m_kd.set(m_kd.get());

        NetworkTable heights = table.getSubTable("Heights");
        m_level1Height = heights.getDoubleTopic("level1").getEntry(Constants.Elevator.Heights.kLevel1);
        m_level1Height.set(m_level1Height.get());

        m_level2Height = heights.getDoubleTopic("level2").getEntry(Constants.Elevator.Heights.kLevel2);
        m_level2Height.set(m_level2Height.get());

        m_level3Height = heights.getDoubleTopic("level3").getEntry(Constants.Elevator.Heights.kLevel3);
        m_level3Height.set(m_level3Height.get());

        m_level4Height = heights.getDoubleTopic("level4").getEntry(Constants.Elevator.Heights.kLevel4);
        m_level4Height.set(m_level4Height.get());
    }

    public void updateInputs(ElevatorConfigIOInputs inputs) {
        inputs.maxVelocity = Units.RotationsPerSecond.of(m_maxVel.getAsDouble());
        inputs.maxAccel = Units.RotationsPerSecondPerSecond.of(m_maxAccel.getAsDouble());
        inputs.feedForward.Ks = m_ks.getAsDouble();
        inputs.feedForward.Kg = m_kg.getAsDouble();
        inputs.feedForward.Kv = m_kv.getAsDouble();
        inputs.feedForward.Ka = m_ka.getAsDouble();
        inputs.PID.Kp = m_kp.getAsDouble();
        inputs.PID.Ki = m_ki.getAsDouble();
        inputs.PID.Kd = m_kd.getAsDouble();
        inputs.decelerationProportion = m_decelProp.getAsDouble();

        inputs.heights.level1 = m_level1Height.getAsDouble();
        inputs.heights.level2 = m_level2Height.getAsDouble();
        inputs.heights.level3 = m_level3Height.getAsDouble();
        inputs.heights.level4 = m_level4Height.getAsDouble();
    }
}
