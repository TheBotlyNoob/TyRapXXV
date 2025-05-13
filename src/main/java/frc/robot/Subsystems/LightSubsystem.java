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
  protected final Timer timer;
  protected double lastRecordedTime;
  protected int counter;

  private static LEDPattern blue = LEDPattern.solid(Color.kBlue);
  private static LEDPattern black = LEDPattern.solid(Color.kBlack);
  private static LEDPattern yellow = LEDPattern.solid(Color.kYellow);
  private static LEDPattern purple = LEDPattern.solid(Color.kPurple);
  private static LEDPattern gray = LEDPattern.solid(Color.kGray);
  private static final double[] validID = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
  private static final double[] lowAlgaeID = {6, 8, 10, 21, 19, 17};
  private static final double[] highAlgaeID = {7, 9, 11, 18, 20, 22};

  public LightSubsystem(
      AddressableLED leds,
      AddressableLEDBuffer ledBuf,
      Limelight ll,
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
    this.ll = ll;
    this.coral = coral;
    this.elevator = elevator;
  }

  public enum AlgaeState {
    HIGH,
    LOW,
    NULL;
  };

  public AlgaeState determineAlgaeHeight() {
    double id = LimelightHelpers.getFiducialID(Constants.ID.kFrontLimelightName);
    for (int i = 0; i < lowAlgaeID.length; i++) {
      if (lowAlgaeID[i] == id) {
        return AlgaeState.LOW;
      }
      if (highAlgaeID[i] == id) {
        return AlgaeState.HIGH;
      }
    }
    return AlgaeState.NULL;
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
      black.applyTo(ledBuf);
    }
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
      default:
        black.applyTo(ledBuf);
        break;
    }

    if (timer.get() - lastRecordedTime >= 0.25) {
      lastRecordedTime = timer.get();
      counter++;
    }

    if (canSeeValidTag()) {
      flashColor();
    }

    led.setData(ledBuf);
  }
}
