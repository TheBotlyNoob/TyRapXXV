package frc.robot;

public class Constants {

    public static class ID {
        /** Algae retriever motor ID. */
        public static final int kAlgaeGrabberMotorCANID = 11;

        /** Algae retriever solenoid ID. */
        public static final int kAlgaeGrabberSolenoidCANID1 = 1;
        public static final int kAlgaeGrabberSolenoidCANID2 = 2;
    }

    public static class AlgaeGrabber {
        /** Motor stall detection threshold. */
        public static final double kMotorCurrentThreshold = 21.0;

        /** Motor speed. */
        public static final double kMotorSpeed = 0.2;
    }

    /**
     * TODO: measure this on TyRapXXV and update the constants
     */
    public static class ElevatorHeights {
        public static final double kGround = 0.0;
        public static final double kLevel1 = 1.0;
        public static final double kLevel2 = 2.0;
        public static final double kLevel3 = 3.0;
        public static final double kLevel4 = 4.0;
    }
}
