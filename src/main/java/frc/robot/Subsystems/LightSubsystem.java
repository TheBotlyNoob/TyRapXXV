package frc.robot.Subsystems;

import java.util.HashMap;

import edu.wpi.first.units.FrequencyUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotState;
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

    private static Color blue = Color.kBlue;
    private static Color black = (Color.kBlack);
    private static Color yellow = (Color.kYellow);
    private static Color purple = (Color.kPurple);
    private static Color gray = (Color.kGray);
    private static Color orange = (Color.kOrange);
    private static Color amber = (Color.kOrangeRed);

    private static final double[] validID = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };

    public LightSubsystem(AddressableLED leds, AddressableLEDBuffer ledBuf, Limelight ll,
            CoralSubsystem coral,
            ElevatorSubsystem elevator, ClimberSubsystem climb) {
        leds.start();
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

    public LEDPattern makeScroll(Color a, Color b) {
        HashMap<Integer, Color> colorMap = new HashMap<>();
        for (int i = 0; i < ledBuf.getLength(); i++) {
            if (i % 2 == 0) {
                colorMap.put((Integer) i, a);
            } else {
                colorMap.put((Integer) i, b);
            }
        }
        return LEDPattern.steps(colorMap).scrollAtRelativeSpeed(Frequency.ofBaseUnits(1.0, Units.Hertz));
    }

    @Override
    public void periodic() {
        if (RobotState.isDisabled()) {
            LEDPattern.solid(amber).applyTo(ledBuf);
        } else if (climber.isClimbMode()) {
            makeScroll(Color.kOrange, Color.kBlue).applyTo(ledBuf);
        } else {
            final Color color;
            switch (elevator.getLevelFlag()) {
                case LEVEL1:
                    color = gray;
                    break;
                case LEVEL2:
                    color = purple;
                    break;
                case LEVEL3:
                    color = yellow;
                    break;
                case LEVEL4:
                    color = blue;
                    break;
                case GROUND:
                default:
                    color = orange;
            }

            if (canSeeValidTag()) {
                makeScroll(color, black).applyTo(ledBuf);
            } else {
                LEDPattern.solid(color).applyTo(ledBuf);
            }
        }

        led.setData(ledBuf);
    }
}
