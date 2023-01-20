// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//FROM Mechanical-Advantage/RobotCode2022
package frc.robot.hardware;
import frc.robot.hardware.LEDIO.LedMode;

/**
 * Manages the pattern of the LEDs based on robot state. Note: This is NOT a WPILib subsystem,
 * meaning it cannot be required by commands. This prevents unexpected conflicts between commands,
 * potentially impacting normal robot operation.
 */
//FROM Mechanical-Advantage/RobotCode2022
//TODO LedBlinkin
//TODO Test?
public class Leds {

  private final LEDIO io;
  // Robot state tracking
  private boolean cone = false;
  private boolean cube = false;
  public Leds(LEDIO io) {
    this.io = io;
  }

  /** Updates the current LED mode based on robot state. */
  public void update() {
    // Select LED mode
    LedMode mode = LedMode.DISABLED_NEUTRAL;
      if (cone) {
        mode = LedMode.CONE;
      } else if (cube) {
        mode = LedMode.CUBE;
      } else {
        mode = LedMode.DISABLED_NEUTRAL;
      }
      io.setMode(mode);
}
  public void cube(boolean active) {
    cube = active;
  }
  public void cone(boolean active) {
    cone = active;
  }
}