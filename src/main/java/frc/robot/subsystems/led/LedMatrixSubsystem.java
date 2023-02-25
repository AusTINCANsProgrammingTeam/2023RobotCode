package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.Arrays;
import java.util.Collections;
import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Robot;
import frc.robot.hardware.LedMatrixImages;

public class LedMatrixSubsystem extends SubsystemBase{
    public static final int ledMatrixLenth = 256;

    public static enum MatrixMode {
        CONE, CUBE;
    }

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
        buffer = new AddressableLEDBuffer(ledMatrixLenth);
        leds.setLength(ledMatrixLenth);
        leds.setData(buffer);
        leds.start();
    }

    public void setMode(MatrixMode mode){
        switch (mode) {
            case CUBE: 
              solid(new Color(0.93333334 * ledBrightnessSlider.getDouble(0.2), 0.50980395 * ledBrightnessSlider.getDouble(0.2), 0.93333334 * ledBrightnessSlider.getDouble(0.2)));
            break;
            case CONE:
              solid(new Color(1 * ledBrightnessSlider.getDouble(0.2), 1 * ledBrightnessSlider.getDouble(0.2), 0.0f));
            break;
        }
        leds.setData(buffer);
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

    private void solid(Color color) {
        for (int i = 0; i < ledMatrixLenth; i++) {
            buffer.setLED(i, color);
        }
    }

    public void offLed(){
        solid(new Color(0.0f, 0.0f, 0.0f));
        leds.setData(buffer);
    }

    public void cargoLed(MatrixMode mode){
        setMode(mode);
        leds.setData(buffer);
      }

    public Command goCans(){
        return new SequentialCommandGroup(
                        new InstantCommand(() -> serpentine(LedMatrixImages.three)),
                        new WaitCommand(1),
                        new InstantCommand(() -> serpentine(LedMatrixImages.two)),
                        new WaitCommand(1),
                        new InstantCommand(() -> serpentine(LedMatrixImages.one)),
                        new WaitCommand(1),
                        new InstantCommand(() -> serpentine(LedMatrixImages.gocans)));
        }

    @Override
    public void periodic() {
    }
}  
