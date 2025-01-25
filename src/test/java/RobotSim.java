import java.util.Vector;
import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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
import frc.sim.*;
import frc.robot.Commands.CenterOnTag;
import frc.sim.*;
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
        // Tag 18 coordinates
        SimTarget target = new SimTarget((float)Units.Meters.convertFrom(144, Units.Inches), 
            (float)Units.Meters.convertFrom(158.5, Units.Inches), 0.0f);
        targets.add(target);
        m_limelight = new SimLimelight(m_drive, targets, useLimelightErrors);
        scheduler = CommandScheduler.getInstance();
        SmartDashboard.putData("Field",field);
    }

    @org.junit.jupiter.api.Test
    public void testCase()
    {    
        // Enable the simulated robot
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        nt.startLocal();

        DataLogManager.start("simlogs","SimLog.wpilog");
        runSim(5.0f, 2.0f, 3.0f, 10.0f);
        runSim(5.0f, 2.0f, 3.0f, 0.0f);
        runSim(5.0f, 2.0f, 5.0f, 0.0f);
    }

    public void runSim(float endTimeSec, float startX, float startY, float startYawDeg)
    {
        this.endTimeSec = endTimeSec;

        // Initialize simulated hardware
        Pose3d startPose = new Pose3d(
            startX, startY, 0.0, new Rotation3d(0.0, 0.0, Math.toRadians(startYawDeg))
        );
        m_drive.setSimPose(startPose);
    
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
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
            }
            nt.flushLocal();
            if (cot.isFinished())
            {
                break;
            }
        }
        assertEquals(1, 1);
    }
}
