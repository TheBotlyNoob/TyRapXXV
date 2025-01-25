package frc.sim;

import java.util.Vector;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Subsystems.Limelight;

public class SimLimelight extends Limelight {

    protected Vector<SimTarget> targets;
    protected SimDrivetrain drivetrain;
    protected Pose3d cameraPosOnBot = new Pose3d(0.3, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
    protected boolean useErrors;

    protected final double SIGMA_ROTATION_DEG = 1.0;
    protected final double SIGMA_POSITION_METERS = 0.002;

    public SimLimelight(SimDrivetrain drivetrain, Vector<SimTarget> targets, boolean useErrors)
    {
        this.drivetrain = drivetrain;
        this.targets = targets;
        this.useErrors = useErrors;
    }

    @Override
    public double[] getTargetPoseCameraSpace() {
        // Calculate the target pose in the camera's reference frame

        // First get the camera position
        Transform3d transformOriginToRobot = new Transform3d(new Pose3d(), drivetrain.getSimPose());
        //System.out.println("cameraTransform:" + transformOriginToRobot.toString());

        Pose3d cameraPose = cameraPosOnBot.plus(transformOriginToRobot);
        //System.out.println("cameraPose:" + cameraPose.toString());

        // Now calculate the relative position of the target
        Pose3d targetPosCameraFrame = targets.get(0).getPose3d().relativeTo(cameraPose);
        double[] retVal = new double[] {
            targetPosCameraFrame.getY()*-1, // Camera Z axis is out from camera 
            targetPosCameraFrame.getZ(), // Camera X axis is positive right from camera
            targetPosCameraFrame.getX(),  // Camera Z axis is positive up 
            Math.toDegrees(targetPosCameraFrame.getRotation().getY()*-1),
            Math.toDegrees(targetPosCameraFrame.getRotation().getZ()*-1),
            Math.toDegrees(targetPosCameraFrame.getRotation().getX()*-1)};
        if (this.useErrors)
        {
            // Generate normal random errors and add to measurements
            // Currently assuming no correlation between errors
            retVal[0] = SimUtilities.getGuasian(retVal[0], SIGMA_POSITION_METERS);
            retVal[1] = SimUtilities.getGuasian(retVal[1], SIGMA_POSITION_METERS);
            retVal[2] = SimUtilities.getGuasian(retVal[2], SIGMA_POSITION_METERS);
            retVal[4] = SimUtilities.getGuasian(retVal[4], SIGMA_ROTATION_DEG);
        }
        //System.out.println("targetPosCameraFrame: " + targetPosCameraFrame.toString());
        System.out.println("camera x=" + retVal[0] + " y=" + retVal[1] + " z=" + retVal[2] + " yaw=" + retVal[4]);
        return retVal;
    }
}
