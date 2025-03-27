package frc.robot.Subsystems.elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.Units;
import frc.robot.Constants;

public class ElevatorMotorIOSpark implements ElevatorMotorIO {
    private final SparkFlex motorLeader;
    private final SparkFlex motorFollower;

    public ElevatorMotorIOSpark() {
        motorLeader = new SparkFlex(Constants.MechID.kElevatorFrontCanId, MotorType.kBrushless);
        motorFollower = new SparkFlex(Constants.MechID.kElevatorBackCanId, MotorType.kBrushless);

        SparkBaseConfig motorConf = new SparkFlexConfig();
        motorConf.smartCurrentLimit(60);
        motorConf.idleMode(IdleMode.kBrake);
        motorConf.inverted(true);
        motorConf.apply(Constants.SparkConstants.defaultSignalConf);
        motorLeader.configure(motorConf, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        SparkBaseConfig followerConf = new SparkFlexConfig();
        followerConf.smartCurrentLimit(60);
        followerConf.idleMode(IdleMode.kBrake);
        followerConf.inverted(false);
        followerConf.follow(motorLeader, false);
        followerConf.apply(Constants.SparkConstants.defaultSignalConf);
        motorFollower.configure(followerConf, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        motorLeader.setVoltage(0);

        motorLeader.getEncoder().setPosition(0);
        motorFollower.getEncoder().setPosition(0);
    }

    @Override
    public void updateInputs(ElevatorMotorIOInputs inputs) {
        inputs.leaderRelativeEncoderPosition = Units.Rotations.of(motorLeader.getEncoder().getPosition());
        inputs.leaderRelativeEncoderVelocity = Units.RPM.of(motorLeader.getEncoder().getVelocity());
    }

    @Override
    public void setLeaderVoltage(double voltage) {
        motorLeader.setVoltage(voltage);
    }

    @Override
    public void resetLeaderEncoderPosition() {
        motorLeader.getEncoder().setPosition(0);
    }
}
