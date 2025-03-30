package frc.robot.Subsystems;

import java.util.stream.IntStream;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems.coral.CoralSubsystem;
import frc.robot.Subsystems.elevator.ElevatorSubsystem;
import frc.robot.Subsystems.vision.Vision;

public class LightSubsystem extends SubsystemBase {
    protected final AddressableLED led;
    protected final AddressableLEDBuffer ledBuf;
    protected final Vision vision;
    protected final CoralSubsystem coral;
    protected final ElevatorSubsystem elevator;
    protected final Timer timer;
    protected double lastRecordedTime;
    protected int counter;

    private static LEDPattern blue = LEDPattern.solid(Color.kBlue);
    private static LEDPattern black = LEDPattern.solid(Color.kBlack);
    private static LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    private static LEDPattern purple = LEDPattern.solid(Color.kPurple);
    private static LEDPattern gray = LEDPattern.solid(Color.kGray);

    public LightSubsystem(AddressableLED leds, AddressableLEDBuffer ledBuf, Vision vision,
            CoralSubsystem coral,
            ElevatorSubsystem elevator) {
        leds.start();
        this.timer = new Timer();
        lastRecordedTime = timer.get();
        counter = 0;
        timer.reset();
        timer.start();
        this.led = leds;
        this.ledBuf = ledBuf;
        this.vision = vision;
        this.coral = coral;
        this.elevator = elevator;
    }

    private void flashColor() {
        if (counter % 2 == 0) {
            black.applyTo(ledBuf);
        }
    }

    private boolean isReefId() {
        return vision.isTargetValid(0)
                && Constants.ID.reefAprilIDs.contains(vision.getFiducialID(0));
    }

    @Override
    public void periodic() {
        switch (elevator.getLevelFlag()) {
            case LEVEL1:
                gray.applyTo(ledBuf);
                break;
            case LEVEL2:
                purple.applyTo(ledBuf);
                break;
            case LEVEL3:
                yellow.applyTo(ledBuf);
                break;
            case LEVEL4:
                blue.applyTo(ledBuf);
                break;
            case GROUND:
                black.applyTo(ledBuf);
                break;
        }

        if (timer.get() - lastRecordedTime >= 0.25) {
            lastRecordedTime = timer.get();
            counter++;
        }

        if (isReefId()) {
            flashColor();
        }

        led.setData(ledBuf);
    }
}
