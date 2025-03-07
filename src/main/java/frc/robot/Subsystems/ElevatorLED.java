package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;
import frc.robot.Subsystems.ElevatorSubsystem.ElevatorLevel;

public class ElevatorLED extends SubsystemBase {
    
    protected final ElevatorSubsystem elevator;
    private static final int kLedPort = 0;
    private static final int kLedLength = 5;
    protected AddressableLED led = new AddressableLED(kLedPort);
    protected AddressableLEDBuffer ledBuffer =  new AddressableLEDBuffer(kLedLength);

    public ElevatorLED(ElevatorSubsystem elevator) {
        led.setLength(kLedLength);
        led.setData(ledBuffer);
        led.start();
        this.elevator = elevator;
    }

    public void setToColor(int index, int r, int g, int b) {
        ledBuffer.setRGB(index, r, g, b);
    }

    
    public void changeAllLEDColor(int r, int g, int b) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            setToColor(i, r, g, b);
        }
    }


    @Override
    public void periodic() {
        switch (elevator.getLevel()){
            case GROUND:
                changeAllLEDColor(255, 255, 255);
            case LEVEL1:
                changeAllLEDColor(128, 128, 128);
            case LEVEL2:
                changeAllLEDColor(127, 0, 255);
            case LEVEL3:
                changeAllLEDColor(255, 255, 0);
            case LEVEL4:
                changeAllLEDColor(0, 0, 255);
        }
        led.setData(ledBuffer);
    }

}
