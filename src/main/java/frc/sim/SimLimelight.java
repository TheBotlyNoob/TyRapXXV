package frc.sim;

import java.util.Vector;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Subsystems.Limelight;
import frc.robot.Constants.LimelightConstants;

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

    public Pose3d getCameraPosOnBot() {
        return cameraPosOnBot;
    }

    public Transform3d getCameraTransform() {
        return new Transform3d(new Pose3d(), cameraPosOnBot);
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

        double[] retVal;

        // Check whether the tag is in the field of view
        double azAngleToTargetRad = Math.atan2(targetPosCameraFrame.getY(),targetPosCameraFrame.getX());
        if (Math.abs(Math.toDegrees(azAngleToTargetRad)) > LimelightConstants.azimuthFieldOfViewDeg) {
            // Tag out of field of view
            retVal = new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            System.out.println("Out of field of view az=" + Math.toDegrees(azAngleToTargetRad));
        } else {
            // Store in Limelight output format
            retVal = new double[] {
                targetPosCameraFrame.getY()*-1, // Camera X axis is positive right from camera 
                targetPosCameraFrame.getZ(), // Camera Y axis is positive up from camera
                targetPosCameraFrame.getX(),  // Camera Z axis is positive out from camera 
                Math.toDegrees(targetPosCameraFrame.getRotation().getY()*-1), // Limelight angles are CW positive
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
        }

        //System.out.println("targetPosCameraFrame: " + targetPosCameraFrame.toString());
        System.out.println("camera x=" + retVal[0] + " y=" + retVal[1] + " z=" + retVal[2] + " yaw=" + retVal[4]);
        return retVal;
    }

    public void reset() {
        this.timeSinceValid = 0;
    }
}
