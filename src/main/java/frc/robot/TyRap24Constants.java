package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class TyRap24Constants {

    public static class ID {
        // Swerve motor controller IDs
        public static final int kFrontLeftTurn = 5; // 1
        public static final int kFrontLeftDrive = 6; // 2

        public static final int kFrontRightTurn = 7; // 3
        public static final int kFrontRightDrive = 8; // 4

        public static final int kBackRightTurn = 1; // 5
        public static final int kBackRightDrive = 2; // 6

        public static final int kBackLeftTurn = 3; // 7
        public static final int kBackLeftDrive = 4; // 8

        // Swerve CanCoder IDs
        public static final int kFrontLeftCANCoder = 11; // 9
        public static final int kFrontRightCANCoder = 12; // 10
        public static final int kBackRightCANCoder = 9; // 11
        public static final int kBackLeftCANCoder = 10; // 12

        // Pigeon
        public static final int kGyro = 13;

        // Intake Pickup IDs
        public static final int kFrontPickup = 14;
        public static final int kBackPickup = 15;

        // Shooter motor controller IDs
        public static final int kTopShooterMotorID = 16;
        public static final int kBottomShooterMotorID = 17;
        public static final int kShooterAimMotorID = 18;
        public static final int kRollerMotorID = 19;
    }

    public static class Deadbands {
        public static final double kLeftJoystickDeadband = 0.04;
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

        public static final double kTranslationPathPlannerP = 5; // shouldnt need anything other than P
        public static final double kRotationPathPlannerP = 4.5;
        // ITS TUNED. NO TOUCH!
        public static final double[] turnPID = { 4.5, 0.0, 0.0 };
        public static final double[] drivePID = { 3, 0.00, 0.00 };
        public static final double[] turnFeedForward = { 0.0, 0.3 };
        public static final double[] driveFeedForward = { 0.0, 2.675 };

        public static final boolean kInvertTurn = false;
        public static final double kMaxPossibleSpeed = 5.0; // meters per second
    };

    public static final class Modules {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75; // check 0 to auto for this (lucas will understand)
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    }

    public static class Controller {
        public static final int kDriveControllerID = 0;
        public static final int kManipControllerID = 1;

        /**
         * Rate limiters make joystick inputs more gentle; 1/3 sec from 0 to 1.
         */
        public static final double kRateLimitXSpeed = 100.0;
        public static final double kRateLimitYSpeed = 100.0;
        public static final double kRateLimitRot = 70.0;
        public static final double kMaxNecessarySpeed = DriveTrainConstants.kMaxPossibleSpeed * 0.8;

        public static final CommandXboxController kDriveController = new CommandXboxController(kDriveControllerID);
    }

    public static class Offsets {
        // Ensure that the encoder offsets are between -Pi & Pi
        /**
         * Encoder offsets
         */
        public static final double kFrontLeftOffset = 0.86; // -0.43
        public static final double kFrontRightOffset = -1.53; // -1.61
        public static final double kBackLeftOffset = -1.61; // -1.53
        public static final double kBackRightOffset = -0.43; // 0.86
    }

    public static class LEDs {
        public static final int kLEDBufferLength = 44;
        public static final int kpwm = 9;
    }
}
