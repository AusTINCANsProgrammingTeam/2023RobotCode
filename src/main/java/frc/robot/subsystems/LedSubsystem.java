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

public class LedSubsystem extends SubsystemBase {
  private PWM pwm;
  private final AddressableLEDBuffer buffer;
  // TODO Led parameters and RIO ports
  // TODO Add to ShuffleBoard
  private static final int length = 1; //Amount of Leds in strip light. Or one if we are using a single light
  private static final int port = 0;
  private final AddressableLED leds = new AddressableLED(port);
  private static enum LedMode {
    CONE, CUBE;
  }
  private LedMode ledMode = LedMode.CONE;
  /** Creates a new Leds. */
  public LedSubsystem() {
    LedDriver(1); //Set channel
    buffer = new AddressableLEDBuffer(length);
    leds.setLength(length);
    leds.setData(buffer);
  }
  /** Sends to Led blinkin */
  public void setMode(LedMode mode){
      switch (mode) {
          case CUBE: 
            solid(Color.kPurple);
            System.out.println("I am Cube");
          break;
          case CONE:
            solid(Color.kYellow);
            System.out.println("I am Cone");
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

  /* Sets Pwm Channel */
  public void LedDriver(int channel) {
    pwm = new PWM(channel);
    pwm.setBounds(1,1, 1, 1, 1); /*TODO Setbounds */
    pwm.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
  }

  public void stopLed(){
    leds.stop();
  }
  public void startLed(){
    leds.start();
  }
  public void changeGamePiece(){
    if (ledMode == LedMode.CONE){
      ledMode = LedMode.CUBE;
    } else if (ledMode == LedMode.CUBE){
      ledMode = LedMode.CONE;
    }
  }

  @Override
  public void periodic() {
    setMode(ledMode);
  }
}
