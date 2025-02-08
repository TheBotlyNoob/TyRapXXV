package frc.robot;

public class Constants {
    
    public static class LimelightConstants {
        // For CenterOnTag
        public static final double minXVelocity = 0.1;
        public static final double maxXVelocity = 1.0;
        public static final double minYVelocity = 0.1;
        public static final double maxYVelocity = 1.0;
        public static final double minAngVelocityDPS = 0;
        public static final double maxAngVelocityDPS = 20;
        public static final double xDisThreshold = 0.03;
        public static final double yDisThreshold = 0.03;
        public static final double rotThreshold = 1.0;
        public static final double azimuthFieldOfViewDeg = 29.0;
        public static final double xOffset = 0.0;
        public static final double yOffset = 0.5;
        public static final double maxAngAccMSS = 8;
        public static final double maxAngDccMSS = 16;
        // For ProportionalController & DriveDistance as well
        public static final double maxAccMSS = 4;
        public static final double maxDccMSS = 3;
        public static final double minVelocity = 0.15;
        public static final double maxVelocity = 5.0;
        public static final double offset = 0.0;
        public static final double proportion = 2;
        public static final double threshold = .02;
        // Camera Positioning
        public static final double cameraOffsetForwardM = 0.19;

    }

    public static class ElevatorConstants {
        public static final double maxHeightM = 2.0;
        public static final double minHeightM = 0.8;
        public static final double maxVelocityMps = 2.0;
    }
}
