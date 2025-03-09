package frc.robot.Subsystems;

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

    private static LEDPattern blue = LEDPattern.solid(Color.kBlue);
    private static LEDPattern black = LEDPattern.solid(Color.kBlack);
    private static LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    private static LEDPattern purple = LEDPattern.solid(Color.kPurple);
    private static LEDPattern gray = LEDPattern.solid(Color.kGray);
    private static LEDPattern orange = LEDPattern.solid(Color.kOrange);
    private static LEDPattern amber = LEDPattern.solid(Color.kOrangeRed);

    private static final double[] validID = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };

    public LightSubsystem(AddressableLED leds, AddressableLEDBuffer ledBuf, Limelight ll,
            CoralSubsystem coral,
            ElevatorSubsystem elevator, ClimberSubsystem climb) {
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

    public LEDPattern makeScroll(LEDPattern a, LEDPattern b) {
        return b.blend(a.scrollAtRelativeSpeed(Frequency.ofBaseUnits(1.0, Units.Hertz)));
    }

    @Override
    public void periodic() {
        if (RobotState.isDisabled()) {
            amber.applyTo(ledBuf);
        } else if (climber.isClimbMode()) {
            makeScroll(orange, blue).applyTo(ledBuf);
        } else {
            final LEDPattern color;
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
                color.applyTo(ledBuf);
            }
        }

        led.setData(ledBuf);
    }
}
