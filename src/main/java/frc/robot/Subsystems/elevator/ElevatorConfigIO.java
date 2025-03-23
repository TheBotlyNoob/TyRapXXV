package frc.robot.Subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

public interface ElevatorConfigIO {
    @AutoLog
    public static class ElevatorConfigIOInputs {
        @AutoLog
        public static class FeedForwardInputs {
            public double Ks = Constants.Elevator.FF.kS;
            public double Kg = Constants.Elevator.FF.kG;
            public double Kv = Constants.Elevator.FF.kV;
            public double Ka = Constants.Elevator.FF.kA;
        }

        @AutoLog
        public static class PIDInputs {
            public double Kp = Constants.Elevator.PID.kP;
            public double Ki = Constants.Elevator.PID.kI;
            public double Kd = Constants.Elevator.PID.kD;
        }

        @AutoLog
        public static class LevelHeights {
            public double level1 = Constants.Elevator.Heights.kLevel1;
            public double level2 = Constants.Elevator.Heights.kLevel2;
            public double level3 = Constants.Elevator.Heights.kLevel3;
            public double level4 = Constants.Elevator.Heights.kLevel4;
        }

        public FeedForwardInputsAutoLogged feedForward = new FeedForwardInputsAutoLogged();
        public PIDInputsAutoLogged PID = new PIDInputsAutoLogged();
        public LevelHeightsAutoLogged heights = new LevelHeightsAutoLogged();

        public AngularVelocity maxVelocity = Units.RotationsPerSecond.of(0.0);
        public AngularAcceleration maxAccel = Units.RotationsPerSecondPerSecond.of(0.0);
        public double decelerationProportion = Constants.Elevator.kDecelProp;
    }

    public default void updateInputs(ElevatorConfigIOInputs inputs) {
    }
}
