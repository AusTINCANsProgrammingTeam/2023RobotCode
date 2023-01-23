// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.LedToggleCommand;

public class LedSubsystem extends SubsystemBase {
  private ShuffleboardTab ledTab = Shuffleboard.getTab("Led");
  private GenericEntry ledState = ledTab.add("OnOff", LedToggleCommand.LedToggle).getEntry();
  private SimpleWidget ledMode = ledTab.add("Color", LedGPMode).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("colorWhenFalse", "Purple", "colorWhenTrue", "Yellow"));
  private PWM pwm;
  public static boolean LedGPMode = true; //True = Cone, False = Cube
  private final AddressableLEDBuffer buffer;
  /** TODO Led parameters and RIO ports */
  private static final int length = 0;
  private static final int port = 0;
  public final static AddressableLED leds = new AddressableLED(port);
  public static enum LedMode {
    CUBE, CONE
  }
  /** Creates a new Leds. */
  public LedSubsystem() {
    LedDriver(1); //Setchannel
    buffer = new AddressableLEDBuffer(length);
    leds.setLength(length);
    leds.setData(buffer);
  }
  /**Updates mode*/    
  public void update() {
    LedMode ledmode;
      // Select LED mode
      if (LedGPMode) {
        ledmode = LedMode.CONE;
      } else {
        ledmode = LedMode.CUBE;
      }
      setMode(ledmode);
    }
  /** Sends to Led blinkin */
  public void setMode(LedMode mode){
      switch (mode) {
          case CUBE: 
            solid(Color.kPurple);
          break;
          case CONE:
            solid(Color.kYellow);
          break;
      }
      leds.setData(buffer);
      pwm.setSpeed(-0.99);
  }
  /* Toggles Mode */
  private void solid(Color color) {
    for (int i = 0; i < length; i++) {
        buffer.setLED(i, color);
    }
  }
  public void LedDriver(int channel) { //Creates PWM channel for Led.
    pwm = new PWM(channel);
    pwm.setBounds(1,1, 1, 1, 1); /*TODO Setbounds */
    pwm.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
  }
  @Override
  public void periodic() {
    update();
    ledState.setBoolean(LedToggleCommand.LedToggle);
    ledMode.getEntry().setBoolean(LedGPMode);
  }

/* If we are using a light strip

  private static final int centerLed = 0;
  private static final int halfLength = (int) Math.ceil(length / 2.0);

  private static final double waveExponent = 0.4; // Controls the length of the transition
  private static final double waveLength = 40.0; // How many LEDs for a full cycle
  private static final double waveDuration = 0.25; // How long until the cycle repeats
  private static final boolean rainbowMode = false;// My Eyes...
  private static final boolean waveMode = false; //Wave effect

  private void setLedsSymmetrical(int index, Color color) {
    buffer.setLED((centerLed + index) % length, color);
    buffer.setLED(centerLed - index, color);
  }
//Rainbow effect
  private void rainbow(double fullLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / fullLength;
    for (int i = 0; i < halfLength; i++) {
      x += xDiffPerLed;
      x %= 180.0;
      setLedsSymmetrical(i, Color.fromHSV((int) x, 255, 255));
    }
  }
//Wave effect
  private void wave(Color c1, Color c2, double fullLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0
        * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / fullLength;
    for (int i = 0; i < halfLength; i++) {
      x += xDiffPerLed;
      double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      setLedsSymmetrical(i, new Color(red, green, blue));
    }
  }

*/
}
