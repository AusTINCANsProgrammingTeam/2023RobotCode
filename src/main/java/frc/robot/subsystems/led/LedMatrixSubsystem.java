package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;
import java.util.Collections;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.hardware.LedDriver;

public class LedMatrixSubsystem extends SubsystemBase{
    private ShuffleboardTab ledTab = Shuffleboard.getTab("Led");
    private GenericEntry ledBrightnessSlider = ledTab.add("Brightness", 0.2).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
    String[][] colors = LedDriver.gocans;

    public Color hex2Rgb(String colorStr) {
        return new Color(
            Integer.valueOf( colorStr.substring( 1, 3 ), 16 ) * ledBrightnessSlider.getDouble(0.2) / 255,
            Integer.valueOf( colorStr.substring( 3, 5 ), 16 ) * ledBrightnessSlider.getDouble(0.2) / 255,
            Integer.valueOf( colorStr.substring( 5, 7 ), 16 ) * ledBrightnessSlider.getDouble(0.2) / 255 );
    }

private final AddressableLEDBuffer buffer;
private final AddressableLED leds = new AddressableLED(4);

    public LedMatrixSubsystem() {
        buffer = new AddressableLEDBuffer(256);
        leds.setLength(256);
        leds.setData(buffer);
        leds.start();
        ledBrightnessSlider.getDouble(0.2);
    }
    boolean self2 = true;
    public void setLed(){
        if (self2){
        for (int i=0; i<colors.length; i += 2) {
            Collections.reverse(Arrays.asList(colors[i]));
        };
        self2 = false;
    }
        for (int i=0; i<colors.length; i++) { 
            for (int j=0; j<colors[i].length; j++){
                buffer.setLED((16*i)+j, hex2Rgb(colors[i][j]));
            }
        }
        leds.setData(buffer);
    }
    int instance = 0;
    public void setRainbowLed(){
        for (int j = 0; j < 255; j++) {
            for (int i = 0, h = 0; i < 256; i++, h++) {
              buffer.setHSV(i, (instance + h)/100, 255, (int) (255 * ledBrightnessSlider.getInteger(4))); //Change the divisor in the hue to change the fade speed
            }
            instance = instance + 1;
        }
        leds.setData(buffer);
    }
    @Override
    public void periodic() {
        setLed();
    }
}  
