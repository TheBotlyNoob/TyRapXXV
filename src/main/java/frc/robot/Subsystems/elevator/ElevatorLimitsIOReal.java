package frc.robot.Subsystems.elevator;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ElevatorLimitsIOReal implements ElevatorLimitsIO {
    private final DigitalInput bottomLimit = new DigitalInput(Constants.Elevator.kBottomLimitSwitch);
    private final DigitalInput topLimit = new DigitalInput(Constants.Elevator.kTopLimitSwitch);

    @Override
    public void updateInputs(ElevatorLimitsIOInputs inputs) {
        // bottom limit switch reversed
        inputs.touchingBottom = !bottomLimit.get();
        inputs.touchingTop = topLimit.get();
    }
}
