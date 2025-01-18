package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SparkJrConstants {
    public static class ID {
        // Swerve motor controller IDs
        public static final int kFrontLeftTurn = 5;
        public static final int kFrontLeftDrive = 6;

        public static final int kFrontRightTurn = 8;
        public static final int kFrontRightDrive = 52;

        public static final int kBackRightTurn = 3;
        public static final int kBackRightDrive = 2;

        public static final int kBackLeftTurn = 4;
        public static final int kBackLeftDrive = 7;

        // Swerve CanCoder IDs
        public static final int kFrontLeftCANCoder = 11;
        public static final int kFrontRightCANCoder = 12;
        public static final int kBackRightCANCoder = 9;
        public static final int kBackLeftCANCoder = 10;

        // Pigeon
        public static final int kGyro = 13;
    }

    public static class Deadbands {
        public static final double kLeftJoystickDeadband = 0.06;
        public static final double kRightJoyStickDeadband = 0.04;
    }

    public static class DriveTrainConstants {
        public static final double kDistanceMiddleToFrontMotor = 0.314325;
        public static final double kDistanceMiddleToSideMotor = 0.314325;
        public static final double kDriveBaseRadius = Math.sqrt( // distance from the middle to the furthest wheel
                kDistanceMiddleToFrontMotor * kDistanceMiddleToFrontMotor +
                        kDistanceMiddleToSideMotor * kDistanceMiddleToSideMotor);

        public static final int kXForward = 1;
        public static final int kXBackward = -1;
        public static final int kYLeft = 1;
        public static final int kYRight = -1;

        // ITS TUNED. NO TOUCH!
        public static final double[] turnPID = { 1.5, 0.2, 0.0 };
        public static final double[] drivePID = { 3, 0.1, 0.0 };
        public static final double[] turnFeedForward = { 0.0, 0.3 };
        public static final double[] driveFeedForward = { 0.0, 2.675 };

        public static final boolean kInvertTurn = true;
        public static final double kMaxPossibleSpeed = 1.5; // meters per second
    };

    public static final class Modules {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    }

    public static class Controller {
        public static final int kDriveControllerID = 1;
        public static final int kManipControllerID = 0;

        /**
         * Rate limiters make joystick inputs more gentle; 1/3 sec from 0 to 1.
         */
        public static final double kRateLimitXSpeed = 100.0;
        public static final double kRateLimitYSpeed = 100.0;
        public static final double kRateLimitRot = 70.0;
        public static final double kMaxNecessarySpeed = DriveTrainConstants.kMaxPossibleSpeed * 0.8;

        public static final CommandXboxController kDriveController = new CommandXboxController(kDriveControllerID);
        public static final CommandXboxController kManipulatorController = new CommandXboxController(
                kManipControllerID);
    }

    public static class Offsets {
        // Ensure that the encoder offsets are between -Pi & Pi
        /**
         * Encoder offsets. These can be obtained by straightening
         * all wheels in the forward position and reading the encoder values 
         * from the dashboard by uncommenting the lines in Robot.java
         * disabledPeriodic.
         */
        public static final double kFrontLeftOffset = 2.79;
        public static final double kFrontRightOffset = 2.106;
        public static final double kBackLeftOffset = 1.10;
        public static final double kBackRightOffset = -0.08;
    }
}
