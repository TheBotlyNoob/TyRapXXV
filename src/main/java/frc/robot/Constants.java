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
        // For DriveOffset
        public static final double driveOffsetXOffset = 0.5;
        public static final double driveOffsetYOffset = 0.3;
        public static final double driveOffsetMaxAccMSS = 5;
        public static final double driveOffsetMaxDccMSS = 5;
        public static final double driveOffsetMinVel = 0.1;
        public static final double driveOffsetMaxVel = 1.5;
        public static final double driveOffsetAngleError = 0.03;
        public static final double driveOffsetRangeMThreshold = 0.02;
        // Camera Positioning
        public static final double cameraOffsetForwardM = 0.19;

    }

    public static class ElevatorConstants {
        //sim
        public static final double maxHeightM = 2.0;
        public static final double minHeightM = 0.8;
        public static final double maxVelocityMps = 2.0;

        public static final double kMaxVelocity = 4.0;
        public static final double kMaxAcceleration = 4.0;
        public static final double kMaxVoltage = 3;
        //elevator pid not tuned
        public static final double kp = 1.3;
        public static final double ki = 0.0;
        public static final double kd = 0.7;
        //elevator ff not tuned
        public static final double ks = 1.1; //static friction (V)
        public static final double kg = 1.2; //gravity (V)
        public static final double kv = 1.3; //volts per velocity (V/(m/s))
        public static final double ka = 0.0; //volts per acceleration (V/(m/s^2))
        //fake values
        public static final double kMaxElevatorHeight = 1.75; //determine units
        public static final double kMinElevatorHeight = 0.0;
        public static double[] HEIGHT_STAGE = {0, 1, 2, 3};
        public static final double kElevatorGearing = 10;
        public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
        public static final double kCarriageMass = 4.0; // kg
        //demoboard IDs
        public static final int leadCANID = 13; 
        public static final int followCANID = 14;

    }
}
