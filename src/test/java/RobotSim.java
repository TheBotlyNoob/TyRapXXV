package frc.sim;

import java.util.Vector;
import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.RawLogEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.BeforeEach;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.Commands.CenterOnTag;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DataLogManager;

public class RobotSim {
    
    protected float endTimeSec = (float)0.0;

    SimDrivetrain m_drive = new SimDrivetrain();
    SimLimelight m_limelight;
    Vector<SimTarget> targets = new Vector<SimTarget>();
    protected final Field2d field = new Field2d();
    protected CommandScheduler scheduler = null;
    protected NetworkTableInstance nt = NetworkTableInstance.getDefault();
    protected final boolean useLimelightErrors = true;

    public RobotSim() {
    }

    @BeforeEach
    public void setup() {
        scheduler = CommandScheduler.getInstance();
    }

    @org.junit.jupiter.api.Test
    public void testCase()
    {
        SmartDashboard.putData("Field",field);
        runSim(5.0f, 2.0f, 1.0f, 20.0f);
    }

    public void runSim(float endTimeSec, float targetX, float targetY, float targetYawDeg)
    {
        this.endTimeSec = endTimeSec;

        // Initialize the sim environment
        SimTarget target = new SimTarget(targetX, targetY, targetYawDeg);
        targets.add(target);
        DataLogManager.start("simlogs","SimLog.wpilog");

        // Initialize simulated hardware
        m_limelight = new SimLimelight(m_drive, targets, useLimelightErrors);

        // Enable the simulated robot
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        nt.startLocal();
    
        // Load command
        CenterOnTag cot = new CenterOnTag(m_drive, m_limelight);
        scheduler.schedule(cot);
    
        // Run simulation loop
        double timeStepSec = 0.02;
        for (double t = 0.0; t < (double)endTimeSec; t+=timeStepSec)
        {
            DriverStationSim.setMatchTime(t);
            scheduler.run();
            field.setRobotPose(m_drive.getSimPose().toPose2d());
            System.out.println("T=" + (float)t + " Robot x=" + m_drive.getSimPose().getX() +
                " y=" + m_drive.getSimPose().getY() + " yaw=" + m_drive.getSimPose().getRotation().getMeasureZ().in(Units.Degrees));
            nt.flushLocal();
            if (cot.isFinished())
            {
                break;
            }
        }
        assertEquals(1, 1);
    }
}
