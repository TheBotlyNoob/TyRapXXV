package frc.sim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class SimTarget {

    protected Pose3d pose = new Pose3d();

    public SimTarget(Pose3d pose) {
        this.pose = pose;
    }

    public SimTarget(float xPos, float yPos, float yawAngleDegrees) {
        this.pose = new Pose3d(xPos, yPos, 0.0,
            new Rotation3d(0.0, 0.0, Math.toRadians(yawAngleDegrees)));  
    }

    public Pose3d getPose3d() {
        return this.pose;
    }
}
