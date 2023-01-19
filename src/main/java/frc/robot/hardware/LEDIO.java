// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//FROM Mechanical-Advantage/RobotCode2022
package frc.robot.hardware;

/** Leds hardware interface. */
public interface LEDIO {

  /** Sets the current LED mode. */
  public default void setMode(LedMode mode) {}

  /**
   * Possible LED modes based on robot state, IO implementations should select an appropriate
   * pattern.
   */
  public static enum LedMode {
    CUBE, CONE, DISABLED_NEUTRAL
  }
}