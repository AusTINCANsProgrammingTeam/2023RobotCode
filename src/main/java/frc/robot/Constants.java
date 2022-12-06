// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class MotorDefaults{
        //Constants to use as default values for Motor Controllers
        public static final int kCurrentLimit = 40;
        public static final double kOpenLoopRampRate = 0.2;
    }


    public static final class DriveConstants{ //TODO: Update these constants with physical values
        public static final double kPhysicalMaxSpeed = 1; //Max drivebase speed in meters per second
        public static final double kPhysicalMaxAngularSpeed = 1; //Max drivebase angular speed in radians per second

        public static final double kTrackWidth = Units.inchesToMeters(18.75); //Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(18.75); //Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics( //Creates robot geometry using the locations of the 4 wheels
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2), 
            new Translation2d(kWheelBase / 2, kTrackWidth /2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2));
    }

    public static final class SwerveModuleConstants{ //TODO: Update these constants with physical values
        public static final double kPhysicalMaxSpeed = 1; //Max module speed in meters per second
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 6.75 / 1;
        public static final double kTurningMotorGearRatio = (150/7) / 1;
        public static final double kDriveEncoderRotFactor = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters; //Conversion factor converting the Drive Encoder's rotations to meters
        public static final double kDriveEncoderRPMFactor = kDriveEncoderRotFactor / 60; //Conversion factor converting the Drive Encoder's RPM to meters per second
        public static final double kTurningEncoderRotFactor = kTurningMotorGearRatio * 2 * Math.PI; //Conversion factor converting the Turn Encoder's rotations to Radians
        public static final double kTurningEncoderRPMFactor = kTurningEncoderRotFactor / 60; //Conersion factor converting the Turn Encoder's RPM to radians per second
        public static final double kPTurning = 0.03; //P gain for the turning motor
    }

    public static final class AutonConstants {
        public static final double kMaxSpeed = DriveConstants.kPhysicalMaxSpeed / 4; //Maximum speed allowed in auton, in meters per second
        public static final double kMaxAcceleration = 3; //Maximum accelaration allowed in auton, in meters per seconds squared

        public static final double kXTranslationP = 1.5;
        public static final double kYTranslationP = 1.5;
        public static final double kRotationP = 3;
    }
}
