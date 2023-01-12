// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Battery Constants
    public static final double minVoltageYellow = 11.0;
    public static final double minVoltageRed = 12.0;
    public static final double minVoltageRedDouble = 10.0;
    public static final int timeInSecondsSimTest = 5;
    public static final double timeInSecondsGeneralYellow = 360.0;
    public static final double timeInSecondsGeneralRed = 600.0;
    public static final double timeInSecondsHighCurrentYellow = 120.0;
    public static final double timeInSecondsHighCurrentRed = 180.0;
    public static final double highBatteryCurrentThreshold = 10;
        
}
