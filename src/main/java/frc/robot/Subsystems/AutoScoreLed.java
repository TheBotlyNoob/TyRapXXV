package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;
import frc.robot.Subsystems.CoralSubsystem;
import frc.robot.Subsystems.CoralSubsystem.CoralState;
import frc.robot.Subsystems.ElevatorSubsystem.ElevatorLevel;

public class AutoScoreLed extends SubsystemBase {
    protected final AddressableLED led;
    protected final AddressableLEDBuffer ledBuf;
    protected final Limelight ll;
    protected final CoralSubsystem coral;
    protected final ElevatorSubsystem elevator;

    private static LEDPattern blue = LEDPattern.solid(Color.kBlue);
    private static LEDPattern black = LEDPattern.solid(Color.kBlack);
    private static LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    private static LEDPattern purple = LEDPattern.solid(Color.kPurple);
    private static LEDPattern gray = LEDPattern.solid(Color.kGray);
    public AutoScoreLed(AddressableLED leds, AddressableLEDBuffer ledBuf, Limelight ll,
            CoralSubsystem coral,
            ElevatorSubsystem elevator) {
        leds.start();

        this.led = leds;
        this.ledBuf = ledBuf;
        this.ll = ll;
        this.coral = coral;
        this.elevator = elevator;
    }

    boolean canAutoScore() {
        // TODO: Decide when the LEDs should be set.
        //return ll.getTimeSinceValid() <= 1 && coral.getState() == CoralState.HOLDING;
        return true;
    }

    @Override
    public void periodic() {
        if (elevator.getLevelFlag() == ElevatorLevel.GROUND) {
            black.applyTo(ledBuf);
        } else if (elevator.getLevelFlag() == ElevatorLevel.LEVEL1) {
            gray.applyTo(ledBuf);
        } else if (elevator.getLevelFlag() == ElevatorLevel.LEVEL2) {
            purple.applyTo(ledBuf);
        } else if (elevator.getLevelFlag() == ElevatorLevel.LEVEL3) {
            yellow.applyTo(ledBuf);
        } else if (elevator.getLevelFlag() == ElevatorLevel.LEVEL4) {
            blue.applyTo(ledBuf);
        }

        led.setData(ledBuf);
    }

}
