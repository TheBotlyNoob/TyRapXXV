import java.util.Vector;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.sim.*;
import frc.robot.Commands.CenterOnTag;
import frc.robot.Commands.DriveDistance;
import frc.robot.Commands.DriveOffset;
import frc.robot.Constants.LimelightConstants;
import frc.sim.*;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.DataLogManager;

public class RobotSim {
    /*
     * protected float endTimeSec = (float) 0.0;
     * protected double TOLERANCE_METERS = 0.01;
     * protected double TOLERANCE_DEGREES = 0.5;
     * 
     * SimDrivetrain m_drive;
     * SimLimelight m_limelight;
     * // SimElevator m_elevator = new SimElevator();
     * // SimCoralManipulator m_CoralManipulator = new
     * SimCoralManipulator(m_elevator);
     * Vector<SimTarget> targets = new Vector<SimTarget>();
     * protected final Field2d field = new Field2d();
     * protected CommandScheduler scheduler = null;
     * protected NetworkTableInstance nt = NetworkTableInstance.getDefault();
     * protected final boolean useLimelightErrors = true;
     * 
     * public RobotSim() {
     * }
     * 
     * // @BeforeEach
     * public void setup() {
     * // Tag 18 coordinates
     * SimTarget target = new SimTarget((float) Units.Meters.convertFrom(144,
     * Units.Inches),
     * (float) Units.Meters.convertFrom(158.5, Units.Inches), 0.0f);
     * targets.add(target);
     * m_limelight = new SimLimelight(m_drive, targets, useLimelightErrors);
     * scheduler = CommandScheduler.getInstance();
     * SmartDashboard.putData("Field", field);
     * }
     * 
     * // @org.junit.jupiter.api.Test
     * public void testCase() {
     * // Enable the simulated robot
     * DriverStationSim.setDsAttached(true);
     * DriverStationSim.setAutonomous(true);
     * DriverStationSim.setEnabled(true);
     * DriverStationSim.notifyNewData();
     * nt.startLocal();
     * 
     * DataLogManager.start("simlogs", "SimLog.wpilog");
     * // runSim(5.0f, 2.0f, 3.0f, 15.0f);
     * runSim(5.0f, 2.0f, 3.5f, 0.0f);
     * runSim(5.0f, 2.0f, 4.5f, 0.0f);
     * runSim(5.0f, 2.0f, 5.0f, -10.0f);
     * runSim(5.0f, 1.0f, 5.0f, -10.0f);
     * }
     * 
     * public void runSim(float endTimeSec, float startX, float startY, float
     * startYawDeg) {
     * this.endTimeSec = endTimeSec;
     * 
     * // Initialize simulated hardware
     * Pose3d startPose = new Pose3d(
     * startX, startY, 0.0, new Rotation3d(0.0, 0.0, Math.toRadians(startYawDeg)));
     * m_drive.setSimPose(startPose);
     * m_limelight.reset();
     * 
     * // Load command
     * CenterOnTag cot = new CenterOnTag(m_drive, m_limelight);
     * scheduler.schedule(cot);
     * 
     * // Run simulation loop
     * double timeStepSec = 0.02;
     * for (double t = 0.0; t < (double) endTimeSec; t += timeStepSec) {
     * DriverStationSim.setMatchTime(t);
     * scheduler.run();
     * field.setRobotPose(m_drive.getSimPose().toPose2d());
     * /*
     * System.out.println("T=" + (float) t + " Robot x=" +
     * m_drive.getSimPose().getX() +
     * " y=" + m_drive.getSimPose().getY() + " yaw="
     * + m_drive.getSimPose().getRotation().getMeasureZ().in(Units.Degrees));
     */
    /*
     * try {
     * Thread.sleep(10);
     * } catch (InterruptedException e) {
     * }
     * nt.flushLocal();
     * if (cot.isFinished()) {
     * break;
     * }
     * }
     * 
     * // Calculate ideal termination conditions
     * Pose3d optimalEndPose = new
     * Pose3d(targets.get(0).getPose3d().getTranslation(),
     * targets.get(0).getPose3d().getRotation());
     * optimalEndPose = optimalEndPose.plus(new Transform3d(
     * -1.0 * LimelightConstants.yOffset,
     * -1.0 * LimelightConstants.xOffset, 0.0, new Rotation3d()));
     * 
     * // Check whether we ended up with tolerances
     * Pose3d finalCameraPos =
     * m_drive.getSimPose().plus(m_limelight.getCameraTransform());
     * Transform3d deltaPos = finalCameraPos.minus(optimalEndPose);
     * System.out.println("Case start x=" + startX + " y=" + startY + " yaw=" +
     * startYawDeg);
     * System.out.println("Final pose: " +
     * finalCameraPos.getTranslation().toString() + " " +
     * Math.toDegrees(finalCameraPos.getRotation().getZ()));
     * System.out.println("Optimal end pose: " + optimalEndPose.getTranslation() +
     * " " +
     * Math.toDegrees(optimalEndPose.getRotation().getZ()));
     * System.out.println("x thresh=" + LimelightConstants.xDisThreshold + " x_act="
     * + deltaPos.getX());
     * // System.out.println("y thresh=" + LimelightConstants.yDisThreshold +
     * " y_act="
     * // + deltaPos.getY());
     * System.out.println("rot thresh=" + LimelightConstants.rotThreshold +
     * " rot_act=" +
     * Math.toDegrees(deltaPos.getRotation().getZ()));
     * assertTrue(Math.abs(deltaPos.getX()) <= (LimelightConstants.xDisThreshold +
     * TOLERANCE_METERS),
     * "X thresh check");
     * // assertTrue(Math.abs(deltaPos.getY()) <= (LimelightConstants.yDisThreshold
     * +
     * // TOLERANCE_METERS),
     * // "Y thresh check");
     * assertTrue(Math.abs(
     * Math.toDegrees(deltaPos.getRotation().getZ())) <=
     * (LimelightConstants.rotThreshold + TOLERANCE_DEGREES),
     * "Angle thresh check");
     * }
     * 
     * // @org.junit.jupiter.api.Test
     * public void testCaseDriveDistance() {
     * // Enable the simulated robot
     * DriverStationSim.setDsAttached(true);
     * DriverStationSim.setAutonomous(true);
     * DriverStationSim.setEnabled(true);
     * DriverStationSim.notifyNewData();
     * nt.startLocal();
     * 
     * DataLogManager.start("simlogs", "SimLog.wpilog");
     * // Drive forward 1m
     * runSimDriveDistance(5.0f, 2.0f, 1.5f, 0.0f, 1.0, 0.0);
     * runSimDriveDistance(5.0f, 2.0f, 1.5f, 0.0f, 0.5, 0.0);
     * runSimDriveDistance(5.0f, 2.0f, 1.5f, 0.0f, 1.0, -90.0);
     * runSimDriveDistance(5.0f, 2.0f, 1.5f, 0.0f, 1.0, +90.0);
     * runSimDriveDistance(5.0f, 2.0f, 1.5f, 0.0f, 1.0, -180.0);
     * runSimDriveDistance(5.0f, 2.0f, 1.5f, 0.0f, 1.0, -45.0);
     * runSimDriveDistance(5.0f, 2.0f, 1.5f, 0.0f, 1.0, 45.0);
     * runSimDriveDistance(5.0f, 2.0f, 1.5f, 0.0f, 1.0, -135.0);
     * runSimDriveDistance(5.0f, 2.0f, 1.5f, 0.0f, 1.0, 135.0);
     * }
     * 
     * public void runSimDriveDistance(float endTimeSec, float startX, float startY,
     * float startYawDeg,
     * double distanceM, double bearingDeg) {
     * this.endTimeSec = endTimeSec;
     * System.out.println("Test Case: dist=" + distanceM + " bearingDeg=" +
     * bearingDeg);
     * // Initialize simulated hardware
     * Pose3d startPose = new Pose3d(
     * startX, startY, 0.0, new Rotation3d(0.0, 0.0, Math.toRadians(startYawDeg)));
     * m_drive.setSimPose(startPose);
     * m_limelight.reset();
     * // m_elevator.reset();
     * // m_elevator.setSpeed(0.5);
     * 
     * // Load command
     * DriveDistance ddc = new DriveDistance(m_drive, () -> distanceM, bearingDeg);
     * 
     * // Run simulation loop
     * double timeStepSec = 0.02;
     * int stepNumber = 0;
     * for (double t = 0.0; t < (double) endTimeSec; t += timeStepSec) {
     * if (stepNumber == 1) {
     * scheduler.schedule(ddc);
     * }
     * 
     * DriverStationSim.setMatchTime(t);
     * scheduler.run();
     * field.setRobotPose(m_drive.getSimPose().toPose2d());
     * System.out.println("T=" + (float) t + " Robot x=" +
     * m_drive.getSimPose().getX() +
     * " y=" + m_drive.getSimPose().getY() + " yaw="
     * + m_drive.getSimPose().getRotation().getMeasureZ().in(Units.Degrees));
     * try {
     * Thread.sleep(5);
     * } catch (InterruptedException e) {
     * }
     * nt.flushLocal();
     * if (stepNumber > 1 && ddc.isFinished()) {
     * break;
     * }
     * stepNumber++;
     * }
     * assertTrue(true);
     * }
     * 
     * // @org.junit.jupiter.api.Test
     * public void testCaseDriveOffset() {
     * // Enable the simulated robot
     * DriverStationSim.setDsAttached(true);
     * DriverStationSim.setAutonomous(true);
     * DriverStationSim.setEnabled(true);
     * DriverStationSim.notifyNewData();
     * nt.startLocal();
     * 
     * DataLogManager.start("simlogs", "SimLog.wpilog");
     * // Drive forward 1m
     * runSimDriveOffset(5.0f, 2.0f, 3.5f, 0.0f, true);
     * runSimDriveOffset(5.0f, 2.0f, 3.5f, 0.0f, false);
     * runSimDriveOffset(5.0f, 2.0f, 3.5f, 10.0f, true);
     * runSimDriveOffset(5.0f, 2.0f, 3.5f, 10.0f, false);
     * runSimDriveOffset(5.0f, 1.0f, 5.0f, -5.0f, true);
     * runSimDriveOffset(5.0f, 1.0f, 5.0f, -5.0f, false);
     * 
     * }
     * 
     * public void runSimDriveOffset(float endTimeSec, float startX, float startY,
     * float startYawDeg, boolean left) {
     * this.endTimeSec = endTimeSec;
     * System.out.println("Test Case: startX=" + startX + " startY=" + startY);
     * // Initialize simulated hardware
     * Pose3d startPose = new Pose3d(
     * startX, startY, 0.0, new Rotation3d(0.0, 0.0, Math.toRadians(startYawDeg)));
     * m_drive.setSimPose(startPose);
     * m_limelight.reset();
     * // m_elevator.reset();
     * // m_elevator.setSpeed(0.5);
     * 
     * // Load command
     * DriveOffset ddc = new DriveOffset(m_drive, m_limelight, left);
     * 
     * // Run simulation loop
     * double timeStepSec = 0.02;
     * int stepNumber = 0;
     * for (double t = 0.0; t < (double) endTimeSec; t += timeStepSec) {
     * if (stepNumber == 1) {
     * scheduler.schedule(ddc);
     * }
     * 
     * DriverStationSim.setMatchTime(t);
     * try {
     * scheduler.run();
     * } catch (Exception e) {
     * e.printStackTrace();
     * break;
     * }
     * 
     * field.setRobotPose(m_drive.getSimPose().toPose2d());
     * System.out.println("T=" + (float) t + " Robot x=" +
     * m_drive.getSimPose().getX() +
     * " y=" + m_drive.getSimPose().getY() + " yaw="
     * + m_drive.getSimPose().getRotation().getMeasureZ().in(Units.Degrees));
     * try {
     * Thread.sleep(5);
     * } catch (InterruptedException e) {
     * }
     * nt.flushLocal();
     * if (stepNumber > 1 && ddc.isFinished()) {
     * break;
     * }
     * stepNumber++;
     * }
     * assertTrue(true);
     * }
     */

}
