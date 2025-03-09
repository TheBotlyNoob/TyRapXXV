package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class LightSubsystem extends SubsystemBase {
    protected final AddressableLED led;
    protected final AddressableLEDBuffer ledBuf;
    protected final Limelight ll;
    protected final CoralSubsystem coral;
    protected final ElevatorSubsystem elevator;
    protected final ClimberSubsystem climber;
    protected final Timer timer;
    protected double lastRecordedTime;
    protected int counter;

    private static LEDPattern blue = LEDPattern.solid(Color.kBlue);
    private static LEDPattern black = LEDPattern.solid(Color.kBlack);
    private static LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    private static LEDPattern purple = LEDPattern.solid(Color.kPurple);
    private static LEDPattern gray = LEDPattern.solid(Color.kGray);
    private static LEDPattern orange = LEDPattern.solid(Color.kOrange);

    private static final double[] validID = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };

    public LightSubsystem(AddressableLED leds, AddressableLEDBuffer ledBuf, Limelight ll,
            CoralSubsystem coral,
            ElevatorSubsystem elevator, ClimberSubsystem climb) {
        leds.start();
        this.timer = new Timer();
        lastRecordedTime = timer.get();
        counter = 0;
        timer.reset();
        timer.start();
        this.led = leds;
        this.ledBuf = ledBuf;
        this.ll = ll;
        this.coral = coral;
        this.elevator = elevator;
        this.climber = climb;
    }

    public boolean canSeeValidTag() {
        double id = LimelightHelpers.getFiducialID(Constants.ID.kFrontLimelightName);
        for (int i = 0; i < validID.length; i++) {
            if (validID[i] == id) {
                // System.out.println("detected reef apriltag id: " + id);
                return true;
            }
        }
        return false;
    }

    public void flashColor() {
        if (counter % 2 == 0) {
            if (!climber.isClimbMode() && canSeeValidTag()) {
                black.applyTo(ledBuf);
            } else if (climber.isClimbMode()) {
                blue.applyTo(ledBuf);
            }
        }
    }

    @Override
    public void periodic() {
        if (climber.isClimbMode()) {
            orange.applyTo(ledBuf);
        } else {
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
                    orange.applyTo(ledBuf);
                    break;
            }
        }

        if (timer.get() - lastRecordedTime >= 0.25) {
            lastRecordedTime = timer.get();
            counter++;
        }

        flashColor();

        led.setData(ledBuf);
    }
}
