package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class AutoScoreLed extends SubsystemBase {
    protected final AddressableLED led;
    protected final AddressableLEDBuffer ledBuf;
    protected final Limelight ll;
    protected final CoralSubsystem coral;
    protected final ElevatorSubsystem elevator;
    protected final Timer timer;

    private static LEDPattern blue = LEDPattern.solid(Color.kBlue);
    private static LEDPattern black = LEDPattern.solid(Color.kBlack);
    private static LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    private static LEDPattern purple = LEDPattern.solid(Color.kPurple);
    private static LEDPattern gray = LEDPattern.solid(Color.kGray);
    private static final int[] validID = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};

    public AutoScoreLed(AddressableLED leds, AddressableLEDBuffer ledBuf, Limelight ll,
            CoralSubsystem coral,
            ElevatorSubsystem elevator) {
        leds.start();
        this.timer = new Timer();
        this.led = leds;
        this.ledBuf = ledBuf;
        this.ll = ll;
        this.coral = coral;
        this.elevator = elevator;
    }

    boolean canSeeValidTag() {
        int id = (int) LimelightHelpers.getFiducialID(Constants.ID.kFrontLimelightName);
        for (int i = 0; i < validID.length; i++){
            if (validID[i] == id){
                return true;
            }
        }
        return false;
    }

    public void flashColor(){
        if (Timer.getFPGATimestamp() % 0.4 == 0){
            black.applyTo(ledBuf);
        }
    }

    @Override
    public void periodic() {
        switch (elevator.getLevelFlag()){
            case LEVEL1:
                gray.applyTo(ledBuf);
                System.out.println("setting level 1 to gray");
                break;
            case LEVEL2:
                purple.applyTo(ledBuf);
                System.out.println("setting level 2 to gray");
                break;
            case LEVEL3:
                yellow.applyTo(ledBuf);
                System.out.println("setting level 3 to gray");
                break;
            case LEVEL4:
                blue.applyTo(ledBuf);
                System.out.println("setting level 4 to gray");
                break;
            case GROUND:
                black.applyTo(ledBuf);
                break;
        }
        if (canSeeValidTag()){
            flashColor();
        }
        led.setData(ledBuf);
    }

}
