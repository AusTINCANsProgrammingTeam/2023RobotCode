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

public class LedMatrixSubsystem extends SubsystemBase{
    private ShuffleboardTab ledTab = Shuffleboard.getTab("Led");
    private GenericEntry ledBrightnessSlider = ledTab.add("Brightness", 0.2)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 1)).getEntry();

    public Color hex2Rgb(String colorStr) {
        return new Color(
            Integer.valueOf( colorStr.substring( 1, 3 ), 16 ) * ledBrightnessSlider.getDouble(0.2) / 255,
            Integer.valueOf( colorStr.substring( 3, 5 ), 16 ) * ledBrightnessSlider.getDouble(0.2) / 255,
            Integer.valueOf( colorStr.substring( 5, 7 ), 16 ) * ledBrightnessSlider.getDouble(0.2) / 255 );
    }

private final AddressableLEDBuffer buffer;
private final AddressableLED leds;

    public LedMatrixSubsystem() {
        leds = new AddressableLED(Robot.ledPort);
        buffer = new AddressableLEDBuffer(Robot.ledMatrixLenth);
        leds.setLength(Robot.ledMatrixLenth);
        leds.setData(buffer);
        leds.start();
        ledBrightnessSlider.getDouble(0.2);
    }

    public void serpentine(String[][] image){
        for (int i=0; i<image.length; i += 2) {
            Collections.reverse(Arrays.asList(image[i]));
        };
        for (int i=0; i<image.length; i++) { 
            for (int j=0; j<image[i].length; j++){
                buffer.setLED((16*i)+j, hex2Rgb(image[i][j]));
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
    }
}  
