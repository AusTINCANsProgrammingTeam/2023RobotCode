package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;

import java.util.Arrays;
import java.util.Collections;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LedMatrixSubsystem {
    public static Color[][] colors = {
        // 'CanMan_Left', 16x16px
{ Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kBlack, Color.kYellow, Color.kYellow, Color.kBlack, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite }, 
{ Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kYellow, Color.kYellow, Color.kBlack, Color.kYellow, Color.kYellow, Color.kBlack, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite }, 
{ Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kYellow, Color.kYellow, Color.kYellow, Color.kBlack, Color.kYellow, Color.kYellow, Color.kYellow, Color.kBlack, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite }, 
{ Color.kWhite, Color.kYellow, Color.kWhite, Color.kWhite, Color.kYellow, Color.kBlack, Color.kBlack, Color.kYellow, Color.kYellow, Color.kBlack, Color.kYellow, Color.kBlack, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite }, 
{ Color.kYellow, Color.kWhite, Color.kYellow, Color.kWhite, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kBlack, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite }, 
{ Color.kBlack, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kBlack, Color.kWhite, Color.kWhite, Color.kWhite }, 
{ Color.kWhite, Color.kWhite, Color.kBlack, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kWhite, Color.kWhite, Color.kWhite }, 
{ Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kWhite, Color.kWhite, Color.kWhite }, 
{ Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kWhite, Color.kWhite, Color.kWhite }, 
{ Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kBlack, Color.kBlack, Color.kYellow, Color.kYellow, Color.kWhite, Color.kWhite, Color.kWhite }, 
{ Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kYellow, Color.kYellow, Color.kBlack, Color.kYellow, Color.kYellow, Color.kYellow, Color.kBlack, Color.kYellow, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite }, 
{ Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite }, 
{ Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kYellow, Color.kYellow, Color.kBlack, Color.kBlack, Color.kYellow, Color.kYellow, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite }, 
{ Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kYellow, Color.kYellow, Color.kBlack, Color.kBlack, Color.kYellow, Color.kYellow, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite }, 
{ Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite }, 
{ Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kBlack, Color.kBlack, Color.kBlack, Color.kYellow, Color.kYellow, Color.kYellow, Color.kBlack, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite }
};

    //Scale color brightness
    private Color scaleColor(Color color, double scale) {
        double scaledRed = color.red / scale;
        double scaledGreen = color.green / scale;
        double scaledBlue = color.blue / scale;
        return new Color(scaledRed, scaledGreen, scaledBlue);
    }

private final AddressableLEDBuffer buffer;
private final AddressableLED leds = new AddressableLED(2);

    public LedMatrixSubsystem() {
        buffer = new AddressableLEDBuffer(256);
        leds.setLength(256);
        leds.setData(buffer);
    }

    public void setLed(){
        for (int i=0; i<colors.length; i += 2) {
            Collections.reverse(Arrays.asList(colors[i]));
        };
        for (int i=0; i<colors.length; i++) { 
            for (int j=0; j<colors[i].length; j++){
                buffer.setLED((16*i)+j, scaleColor(colors[i][j], 0.5));
            }
        }
        leds.setData(buffer);
    }
}