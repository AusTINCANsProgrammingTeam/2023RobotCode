package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LedMatrixSubsystem {
    public static Color[] colors = {
        // 'CanMan_Left', 16x16px
Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kBlack, Color.kYellow, Color.kYellow, Color.kBlack, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, 
Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kYellow, Color.kYellow, Color.kBlack, Color.kYellow, Color.kYellow, Color.kBlack, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, 
Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kYellow, Color.kYellow, Color.kYellow, Color.kBlack, Color.kYellow, Color.kYellow, Color.kYellow, Color.kBlack, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, 
Color.kWhite, Color.kYellow, Color.kWhite, Color.kWhite, Color.kYellow, Color.kBlack, Color.kBlack, Color.kYellow, Color.kYellow, Color.kBlack, Color.kYellow, Color.kBlack, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, 
Color.kYellow, Color.kWhite, Color.kYellow, Color.kWhite, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kBlack, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, 
Color.kBlack, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kBlack, Color.kWhite, Color.kWhite, Color.kWhite, 
Color.kWhite, Color.kWhite, Color.kBlack, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kWhite, Color.kWhite, Color.kWhite, 
Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kWhite, Color.kWhite, Color.kWhite, 
Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kWhite, Color.kWhite, Color.kWhite, 
Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kBlack, Color.kBlack, Color.kYellow, Color.kYellow, Color.kWhite, Color.kWhite, Color.kWhite, 
Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kYellow, Color.kYellow, Color.kBlack, Color.kYellow, Color.kYellow, Color.kYellow, Color.kBlack, Color.kYellow, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, 
Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, 
Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kYellow, Color.kYellow, Color.kBlack, Color.kBlack, Color.kYellow, Color.kYellow, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, 
Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kYellow, Color.kYellow, Color.kBlack, Color.kBlack, Color.kYellow, Color.kYellow, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, 
Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, 
Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kBlack, Color.kBlack, Color.kBlack, Color.kYellow, Color.kYellow, Color.kYellow, Color.kBlack, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite, Color.kWhite
};

private final AddressableLEDBuffer buffer;
private final AddressableLED leds = new AddressableLED(1);

    public LedMatrixSubsystem() {
        buffer = new AddressableLEDBuffer(256);
        leds.setBitTiming(1, 1, 1, 1);
        leds.setLength(256);
        leds.setData(buffer);
    }

    public void test(){
        for (int i=0; i<colors.length; i++) { 
            buffer.setLED(i, colors[i]);
        }
    }
}