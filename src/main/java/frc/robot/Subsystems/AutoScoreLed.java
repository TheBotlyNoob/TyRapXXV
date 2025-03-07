package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;
import frc.robot.Subsystems.CoralSubsystem;
import frc.robot.Subsystems.CoralSubsystem.CoralState;

public class AutoScoreLed extends SubsystemBase {
    protected final AddressableLED led;
    protected final AddressableLEDBuffer ledBuf;
    protected final Limelight ll;
    protected final CoralSubsystem coral;
    protected final ElevatorSubsystem elevator;

    private static LEDPattern canScoreLEDPattern = LEDPattern.solid(Color.kBlue);
    private static LEDPattern cantScoreLEDPattern = LEDPattern.solid(Color.kBlack);

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
        if (canAutoScore()) {
            canScoreLEDPattern.applyTo(ledBuf);
        } else {
            cantScoreLEDPattern.applyTo(ledBuf);
        }

        led.setData(ledBuf);
    }

}
