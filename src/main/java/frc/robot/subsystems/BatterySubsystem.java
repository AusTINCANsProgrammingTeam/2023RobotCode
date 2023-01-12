// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import java.util.Map;

public class BatterySubsystem extends SubsystemBase {

  private double storedGeneralTime;
  private double storedHighCurrentTime;
  private ShuffleboardTab btTab;
  private GenericEntry sbVoltage;
  private GenericEntry sbInputCurrent;
  private GenericEntry sbSimVoltage;
  private GenericEntry sbTimer;
  private GenericEntry sbTimerChange;
  private GenericEntry sbTimerHighCurrent;
  private Timer timer = new Timer();
  private Timer currentTimer = new Timer();
  private PowerDistribution powerDistribution = new PowerDistribution();

  public BatterySubsystem() {

    // Reset timers
    this.resetTimers();
    // Adds tab battery
    btTab = Shuffleboard.getTab("Battery");
    // Color tab for the red voltage level
    Shuffleboard.getTab("Battery")
        .addBoolean("Voltage Red", () -> checkRedVoltage())
        .withPosition(2, 1)
        .withProperties(Map.of("colorWhenFalse", "black"))
        .withProperties(Map.of("colorWhenTrue", "red"));
    // Color tab for the yellow voltage level
    Shuffleboard.getTab("Battery")
        .addBoolean("Voltage Yellow", () -> getVoltage() > Constants.minVoltageYellow)
        .withPosition(3, 1)
        .withProperties(Map.of("colorWhenFalse", "black"))
        .withProperties(Map.of("colorWhenTrue", "yellow"));
    // Color tab for the red timer level
    Shuffleboard.getTab("Battery")
        .addBoolean("Timer Red", () -> getGeneralTimer() > Constants.timeInSecondsGeneralRed)
        .withPosition(2, 2)
        .withProperties(Map.of("colorWhenFalse", "black"))
        .withProperties(Map.of("colorWhenTrue", "light red"));
    // Color tab for the yellow timer level
    Shuffleboard.getTab("Battery")
        .addBoolean("Timer Yellow", () -> getGeneralTimer() > Constants.timeInSecondsGeneralYellow)
        .withPosition(3, 2)
        .withProperties(Map.of("colorWhenFalse", "black"))
        .withProperties(Map.of("colorWhenTrue", "light yellow"));
    // Color tab for the red high current timer level
    Shuffleboard.getTab("Battery")
        .addBoolean("HC Red", () -> getHighCurrentTimer() > Constants.timeInSecondsHighCurrentRed)
        .withPosition(2, 3)
        .withProperties(Map.of("colorWhenFalse", "black"))
        .withProperties(Map.of("colorWhenTrue", "red"));
    // Color tab for the yellow high current timer level
    Shuffleboard.getTab("Battery")
        .addBoolean(
            "HC Yellow", () -> getHighCurrentTimer() > Constants.timeInSecondsHighCurrentYellow)
        .withPosition(3, 3)
        .withProperties(Map.of("colorWhenFalse", "black"))
        .withProperties(Map.of("colorWhenTrue", "yellow"));
    // Adds voltage reader tab
    sbVoltage = btTab.add("Battery Voltage", 0).withSize(2, 1).withPosition(2, 0).getEntry();
    // Adds current reader tab
    sbInputCurrent =
        btTab.add("Battery Input Current", 0).withSize(2, 1).withPosition(4, 0).getEntry();
    // Adds sim voltage reader tab
    sbSimVoltage = btTab.add("Simulation Voltage", 0).withSize(2, 1).withPosition(0, 0).getEntry();
    // Adds general timer reader tab
    sbTimer = btTab.add("Timer", 0).withSize(2, 1).withPosition(0, 2).getEntry();
    // Adds high current timer reader tab
    sbTimerHighCurrent =
        btTab.add("High Current Timer", 0).withSize(2, 1).withPosition(0, 1).getEntry();
    // Adds timer change reader tab
    sbTimerChange = btTab.add("Change Timer", 0).withSize(0, 1).withPosition(4, 0).getEntry();
    // Starts general timer
    timer.start();
    DriverStationSim.setSendError(true);
  }

  public void periodic() {
    // Sets shuffleboard tabs to their respective values
    sbVoltage.setDouble(getVoltage());
    sbInputCurrent.setDouble(getInputCurrent());
    sbSimVoltage.setDouble(powerDistribution.getVoltage());
    sbTimer.setDouble(getGeneralTimer());
    sbTimerHighCurrent.setDouble(getHighCurrentTimer());
    sbTimerChange.setBoolean(checkTimer());
    checkCurrent();
    checkVoltage();
  }

  public double getVoltage() {
    return powerDistribution.getVoltage();
  }

  public double getInputCurrent() {
    if (Robot.isSimulation()) {
      return powerDistribution.getCurrent(1);
    } else {
      return powerDistribution.getTotalCurrent();
    }
  }

  // If the current is below a certain level, stops the high current timer
  public void checkCurrent() {
    if (getInputCurrent() < Constants.highBatteryCurrentThreshold) {
      currentTimer.stop();
    } else {
      currentTimer.start();
    }
  }

  // Displays alerts based on the state of the timer
  // HCTR = High Current Timer Red
  // GTR = General Timer Red
  // HCTY = High Current Timer Yellow
  // GTY = General Timer Yellow
  public boolean checkTimer() {
    if (currentTimer.hasElapsed(Constants.timeInSecondsHighCurrentRed)) {
      DriverStation.reportWarning("Change the Battery Now! (HCTR)", false);
      return true;
    } else if (timer.hasElapsed(Constants.timeInSecondsGeneralRed)) {
      DriverStation.reportWarning("Change the Battery Now! (GTR)", false);
      return true;
    } else if (currentTimer.hasElapsed(Constants.timeInSecondsHighCurrentYellow)) {
      DriverStation.reportWarning("Change the Battery Soon! (HCTY)", false);
      return false;
    } else if (timer.hasElapsed(Constants.timeInSecondsGeneralYellow)) {
      DriverStation.reportWarning("Change the Battery Soon! (GTY)", false);
      return false;
    }
    return false;
  }

  public double getGeneralTimer() {
    Preferences.setDouble("Battery General Timer", timer.get() + storedGeneralTime);
    return timer.get() + storedGeneralTime;
  }

  public double getHighCurrentTimer() {
    Preferences.setDouble("Battery High Current Timer", currentTimer.get() + storedHighCurrentTime);
    return currentTimer.get() + storedHighCurrentTime;
  }

  public void checkVoltage() {
    if (getVoltage() < Constants.minVoltageRed) {
      DriverStation.reportWarning("Change the Battery Now! (VR)", false);
    } else if (getVoltage() < Constants.minVoltageYellow) {
      DriverStation.reportWarning("Change the Battery Soon! (VY)", false);
    } else {
    }
  }

  public boolean checkRedVoltage() {
    return getVoltage() > Constants.minVoltageRed;
  }

  public void resetTimers() {
    // Note: Keeping timer values only works across redeploys, reboots always set them to 0

    // Add a key to store timer values if there isn't one already
    if (!Preferences.containsKey("Battery General Timer")) {
      Preferences.setDouble("Battery General Timer", 0.0);
      Preferences.setDouble("Battery High Current Timer", 0.0);
    } else {
      storedGeneralTime = Preferences.getDouble("Battery General Timer", 0.0);
      storedHighCurrentTime = Preferences.getDouble("Battery High Current Timer", 0.0);
    }
    timer.reset();
    currentTimer.reset();
  }
}
