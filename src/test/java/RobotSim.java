package frc.sim;

import java.util.Vector;
import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    RawLogEntry poseLogEntry;

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
        runSim(2.0f);
    }

    public void runSim(float endTimeSec)
    {
        this.endTimeSec = endTimeSec;

        // Initialize the sim environment
        SimTarget target = new SimTarget(2.0f, 1.0f, 0.0f);
        targets.add(target);
        DataLogManager.start("simlogs","SimLog.wpilog");
        poseLogEntry = new RawLogEntry(DataLogManager.getLog(), "RobotPose");

        // Initialize simulated hardware
        m_limelight = new SimLimelight(m_drive, targets);

        // Enable the simulated robot
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        nt.startLocal();
    
        // Load command
        scheduler.schedule(new CenterOnTag(m_drive, m_limelight));
        int x = 0;
    
        // Run simulation loop
        double timeStepSec = 0.02;
        for (double t = 0.0; t < (double)endTimeSec; t+=timeStepSec)
        {
            DriverStationSim.setMatchTime(t);
            scheduler.run();
            field.setRobotPose(m_drive.getSimPose().toPose2d());
            System.out.println("T=" + (float)t + " Robot pose:" + m_drive.getSimPose().toString());
            nt.flushLocal();
            
        }
        assertEquals(1, 1);
    }
}
