package frc.robot.Subsystems.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import frc.robot.Constants;

public class ElevatorConfigIONetworkTables implements ElevatorConfigIO {
    private final DoubleSupplier m_maxVel;
    private final DoubleSupplier m_maxAccel;
    private final DoubleSupplier m_ks;
    private final DoubleSupplier m_kg;
    private final DoubleSupplier m_kv;
    private final DoubleSupplier m_ka;
    private final DoubleSupplier m_kp;
    private final DoubleSupplier m_ki;
    private final DoubleSupplier m_kd;
    private final DoubleSupplier m_decelProp;

    private final DoubleSupplier m_level1Height;
    private final DoubleSupplier m_level2Height;
    private final DoubleSupplier m_level3Height;
    private final DoubleSupplier m_level4Height;

    public ElevatorConfigIONetworkTables(NetworkTableInstance nt) {
        NetworkTable table = nt.getTable("ElevatorSubsystemConfig");
        m_maxVel = table.getDoubleTopic("maxVelocityRotPerSec").subscribe(Constants.Elevator.kMaxVelocity);
        m_maxAccel = table.getDoubleTopic("maxAccelRotSecPerSec").subscribe(Constants.Elevator.kMaxAcceleration);
        m_decelProp = table.getDoubleTopic("decelProp").subscribe(Constants.Elevator.kDecelProp);

        NetworkTable feedForward = table.getSubTable("FeedForward");
        m_ks = feedForward.getDoubleTopic("ks").subscribe(Constants.Elevator.FF.kS);
        m_kg = feedForward.getDoubleTopic("kg").subscribe(Constants.Elevator.FF.kG);
        m_kv = feedForward.getDoubleTopic("kv").subscribe(Constants.Elevator.FF.kV);
        m_ka = feedForward.getDoubleTopic("ka").subscribe(Constants.Elevator.FF.kA);

        NetworkTable PID = table.getSubTable("PID");
        m_kp = PID.getDoubleTopic("kp").subscribe(Constants.Elevator.PID.kP);
        m_ki = PID.getDoubleTopic("ki").subscribe(Constants.Elevator.PID.kI);
        m_kd = PID.getDoubleTopic("kd").subscribe(Constants.Elevator.PID.kD);

        NetworkTable heights = table.getSubTable("Heights");
        m_level1Height = heights.getDoubleTopic("level1").subscribe(Constants.Elevator.Heights.kLevel1);
        m_level2Height = heights.getDoubleTopic("level2").subscribe(Constants.Elevator.Heights.kLevel2);
        m_level3Height = heights.getDoubleTopic("level3").subscribe(Constants.Elevator.Heights.kLevel3);
        m_level4Height = heights.getDoubleTopic("level4").subscribe(Constants.Elevator.Heights.kLevel4);
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
