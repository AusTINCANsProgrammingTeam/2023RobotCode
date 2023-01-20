// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import java.util.HashMap;
import java.util.Map;

import frc.robot.hardware.LedDriver;
import frc.robot.hardware.LedDriver.LedDMode;

/** Maps LED mode to Blinkin patterns. */
public class LedBlinkin implements LEDIO {
  private static final Map<LedMode, LedDMode> modeLookup =
      new HashMap<>();

  static {
    modeLookup.put(LedMode.CONE, LedDMode.CONE);
    modeLookup.put(LedMode.CUBE, LedDMode.CUBE);
    modeLookup.put(LedMode.DISABLED_NEUTRAL, LedDMode.DISABLED_NEUTRAL);
  }

  private LedDriver blinkin;

  public void LedsIO() {
    blinkin = new LedDriver(0);
  }

  public void setMode(LedDMode mode) {
    if (modeLookup.containsKey(mode)) {
      blinkin.setMode(modeLookup.get(mode));
    } else {
      blinkin.setMode(LedDMode.DISABLED_NEUTRAL);
    }
  }
}