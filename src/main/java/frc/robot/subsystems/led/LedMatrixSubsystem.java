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
import frc.robot.Robot;
import frc.robot.hardware.LedDriver;

public class LedMatrixSubsystem extends SubsystemBase{
    private ShuffleboardTab ledTab = Shuffleboard.getTab("Led");
    private GenericEntry ledBrightnessSlider = ledTab.add("Brightness", 0.2).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
    String[][] colors = LedDriver.gocans;
    String[][] gocans = LedDriver.gocans;
    String[][] one = LedDriver.one;
    String[][] two = LedDriver.two;
    String[][] three = LedDriver.three;

    public Color hex2Rgb(String colorStr) {
        return new Color(
            Integer.valueOf( colorStr.substring( 1, 3 ), 16 ) * ledBrightnessSlider.getDouble(0.2) / 255,
            Integer.valueOf( colorStr.substring( 3, 5 ), 16 ) * ledBrightnessSlider.getDouble(0.2) / 255,
            Integer.valueOf( colorStr.substring( 5, 7 ), 16 ) * ledBrightnessSlider.getDouble(0.2) / 255 );
    }

private final AddressableLEDBuffer buffer;
private final AddressableLED leds = new AddressableLED(Robot.ledPort);

    public LedMatrixSubsystem() {
        buffer = new AddressableLEDBuffer(256);
        leds.setLength(256);
        leds.setData(buffer);
        leds.start();
        ledBrightnessSlider.getDouble(0.2);
    }
    boolean self = true;
    public void setLed(){
        if (self){
        for (int i=0; i<colors.length; i += 2) {
            Collections.reverse(Arrays.asList(colors[i]));
        };
        self = false;
    }
        for (int i=0; i<colors.length; i++) { 
            for (int j=0; j<colors[i].length; j++){
                buffer.setLED((16*i)+j, hex2Rgb(colors[i][j]));
            }
        }
        leds.setData(buffer);
    }
    boolean self2 = true;
    public void setLedOne(){
        if (self2){
            for (int i=0; i<colors.length; i += 2) {
                Collections.reverse(Arrays.asList(one[i]));
            };
            self2 = false;
        }
            for (int i=0; i<colors.length; i++) { 
                for (int j=0; j<colors[i].length; j++){
                    buffer.setLED((16*i)+j, hex2Rgb(one[i][j]));
                }
            }
            leds.setData(buffer);
    }
    boolean self3 = true;
    public void setLedTwo(){
        if (self3){
            for (int i=0; i<colors.length; i += 2) {
                Collections.reverse(Arrays.asList(two[i]));
            };
            self3 = false;
        }
            for (int i=0; i<colors.length; i++) { 
                for (int j=0; j<colors[i].length; j++){
                    buffer.setLED((16*i)+j, hex2Rgb(two[i][j]));
                }
            }
            leds.setData(buffer);
    }
    boolean self4 = true;
    public void setLedThree(){
        if (self4){
            for (int i=0; i<colors.length; i += 2) {
                Collections.reverse(Arrays.asList(three[i]));
            };
            self4 = false;
        }
            for (int i=0; i<colors.length; i++) { 
                for (int j=0; j<colors[i].length; j++){
                    buffer.setLED((16*i)+j, hex2Rgb(three[i][j]));
                }
            }
            leds.setData(buffer);
    }
    boolean self5 = true;
    public void setLedGoCans(){
        if (self5){
            for (int i=0; i<colors.length; i += 2) {
                Collections.reverse(Arrays.asList(gocans[i]));
            };
            self5 = false;
        }
            for (int i=0; i<colors.length; i++) { 
                for (int j=0; j<colors[i].length; j++){
                    buffer.setLED((16*i)+j, hex2Rgb(gocans[i][j]));
                }
            }
            leds.setData(buffer);
    }

    int instance = 0;
    public void setRainbowLed(){
        double bright = ledBrightnessSlider.getDouble(0.2);
        for (int i = 0; i < 256; i++) {
            if (i%2==0 && instance%3==0){
              buffer.setHSV(i, (instance + i)/100, 255, (int) (255 * bright)); //Change the divisor in the hue to change the fade speed
            } else {
                buffer.setHSV(i, 0, 0, 0);
            }
            instance = instance + 1;
        }
        leds.setData(buffer);
    }



    @Override
    public void periodic() {
        setRainbowLed();
    }
}  
