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

    public SimLimelight(SimDrivetrain drivetrain, Vector<SimTarget> targets)
    {
        this.drivetrain = drivetrain;
        this.targets = targets;
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
            Math.toDegrees(targetPosCameraFrame.getRotation().getZ()),
            Math.toDegrees(targetPosCameraFrame.getRotation().getX())};
        //System.out.println("targetPosCameraFrame: " + targetPosCameraFrame.toString());
        System.out.println("camera x=" + retVal[0] + " y=" + retVal[1] + " z=" + retVal[2] + " yaw=" + retVal[4]);
        return retVal;
    }
}
