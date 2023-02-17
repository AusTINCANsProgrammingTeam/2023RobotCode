// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
public class LedSubsystem extends SubsystemBase {
  private final AddressableLEDBuffer buffer;
  private final AddressableLED leds;
  private ShuffleboardTab ledTab = Shuffleboard.getTab("Led");
  private GenericEntry ledState = ledTab.add("OnOff", false).getEntry();
  private SimpleWidget ledGamePiece = ledTab.add("Color", true).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("colorWhenFalse", "Purple", "colorWhenTrue", "Yellow"));
  private double ledBrightnessSlider = ledTab.add("Brightness", 0.2).withWidget(BuiltInWidgets.kNumberSlider).getEntry().getDouble(0.2);

  public static enum LedMode {
    CONE, CUBE, OFF;
  }
  private LedMode ledMode = LedMode.CONE;
  /** Configure AddressableLED */
  public LedSubsystem() {
    leds = new AddressableLED(Robot.ledPort);
    buffer = new AddressableLEDBuffer(Robot.ledStripLength);
    //leds.setBitTiming(1, 1, 1, 1);
    leds.setLength(Robot.ledStripLength);
    leds.setData(buffer);
    leds.start();
    }
  /** Sends to AddressableLED */
  public void setMode(LedMode mode){
      switch (mode) {
          case CUBE: 
            solid(new Color(0.93333334 * ledBrightnessSlider, 0.50980395 * ledBrightnessSlider, 0.93333334 * ledBrightnessSlider));
          break;
          case CONE:
            solid(new Color(1 * ledBrightnessSlider, 1 * ledBrightnessSlider, 0.0f));
          break;
          case OFF:
            solid(new Color(0.0f, 0.0f, 0.0f));
          break;
      }
      leds.setData(buffer);
  }

  /* Solid Color */
  private void solid(Color color) {
    for (int i = 0; i < Robot.ledStripLength; i++) {
        buffer.setLED(i, color);
    }
  }

  public void offLed(){
    ledMode = LedMode.OFF;
    setMode(ledMode);
    ledState.setBoolean(false);
    leds.setData(buffer);
  }
  public void onLed(){
    ledMode = LedMode.CONE;
    setMode(ledMode);
    ledState.setBoolean(true);
    leds.setData(buffer);
  }

  public void changeGamePiece(){
    if (ledMode == LedMode.CONE){
      ledMode = LedMode.CUBE;
      ledGamePiece.getEntry().setBoolean(false);
    } else if (ledMode == LedMode.CUBE){
      ledMode = LedMode.CONE;
      ledGamePiece.getEntry().setBoolean(true);
    }
    setMode(ledMode);
    leds.setData(buffer);
  }

  @Override
  public void periodic() {
  }
}
