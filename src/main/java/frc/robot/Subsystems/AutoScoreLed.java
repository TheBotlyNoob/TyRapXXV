package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;
import frc.robot.Subsystems.CoralSubsystem;
import frc.robot.Subsystems.CoralSubsystem.CoralState;

public class AutoScoreLed extends SubsystemBase {
    protected final AddressableLED led1;
    protected final AddressableLED led2;
    protected final AddressableLEDBuffer ledBuf;
    protected final Limelight ll;
    protected final CoralSubsystem coral;
    protected final ElevatorSubsystem elevator;

    private static LEDPattern canScoreLEDPattern = LEDPattern.solid(Color.kBlue);
    private static LEDPattern cantScoreLEDPattern = LEDPattern.solid(Color.kBlack);

    public AutoScoreLed(AddressableLED led1, AddressableLED led2, AddressableLEDBuffer ledBuf, Limelight ll,
            CoralSubsystem coral,
            ElevatorSubsystem elevator) {
        led1.start();
        led2.start();

        this.led1 = led1;
        this.led2 = led2;
        this.ledBuf = ledBuf;
        this.ll = ll;
        this.coral = coral;
        this.elevator = elevator;
    }

    boolean canAutoScore() {
        // TODO: decide if elevator flag can be unset.
        return ll.getTimeSinceValid() <= 1 && coral.getState() == CoralState.HOLDING;
    }

    @Override
    public void periodic() {
        if (canAutoScore()) {
            canScoreLEDPattern.applyTo(ledBuf);
        } else {
            cantScoreLEDPattern.applyTo(ledBuf);
        }

        led1.setData(ledBuf);
        led2.setData(ledBuf);
    }

}
