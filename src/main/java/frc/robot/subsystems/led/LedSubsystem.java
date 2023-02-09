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
  // TODO Led parameters and RIO ports
public class LedSubsystem extends SubsystemBase {
  private final AddressableLEDBuffer buffer;
  private final AddressableLED leds = new AddressableLED(1);
  private ShuffleboardTab ledTab = Shuffleboard.getTab("Led");
  private GenericEntry ledState = ledTab.add("OnOff", false).getEntry();
  private SimpleWidget ledGamePiece = ledTab.add("Color", true).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("colorWhenFalse", "Purple", "colorWhenTrue", "Yellow"));
  private static final int length = 1; //Amount of Leds in strip light. Or one if we are using a single light
  private static enum LedMode {
    CONE, CUBE;
  }
  private LedMode ledMode = LedMode.CONE;
  /** Configure AddressableLED */
  public LedSubsystem() {
    buffer = new AddressableLEDBuffer(length);
    leds.setBitTiming(1, 1, 1, 1);
    leds.setLength(length);
    leds.setData(buffer);
  }
  /** Sends to AddressableLED */
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
  }

  /* Solid Color */
  private void solid(Color color) {
    for (int i = 0; i < length; i++) {
        buffer.setLED(i, color);
    }
  }

  public void stopLed(){
    leds.stop();
    ledState.setBoolean(false);
  }
  public void startLed(){
    leds.start();
    ledState.setBoolean(true);
  }

  public void changeGamePiece(){
    if (ledMode == LedMode.CONE){
      ledMode = LedMode.CUBE;
      ledGamePiece.getEntry().setBoolean(false);
    } else if (ledMode == LedMode.CUBE){
      ledMode = LedMode.CONE;
      ledGamePiece.getEntry().setBoolean(true);
    }
  }

  @Override
  public void periodic() {
  }
}
