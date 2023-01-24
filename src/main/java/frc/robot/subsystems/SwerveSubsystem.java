// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.AbsoluteEncoder.EncoderConfig;
import frc.robot.hardware.MotorController.MotorConfig;

public class SwerveSubsystem extends SubsystemBase{
    public static final double kPhysicalMaxSpeed = Units.feetToMeters(14.5);; //Max drivebase speed in meters per second
    public static final double kPhysicalMaxAngularSpeed = 2 * Math.PI; //Max drivebase angular speed in radians per second
    public static final double kTrackWidth = Units.inchesToMeters(18.75); //Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(18.75); //Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics( //Creates robot geometry using the locations of the 4 wheels
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), 
        new Translation2d(kWheelBase / 2, -kTrackWidth /2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    private final SwerveModule frontLeft = new SwerveModule(
        MotorConfig.FrontLeftModuleDrive,
        MotorConfig.FrontLeftModuleTurn,
        EncoderConfig.FrontLeftModule,
        "FL");

    private final SwerveModule frontRight = new SwerveModule(
        MotorConfig.FrontRightModuleDrive,
        MotorConfig.FrontRightModuleTurn,
        EncoderConfig.FrontRightModule,
        "FR");

    private final SwerveModule backLeft = new SwerveModule(
        MotorConfig.BackLeftModuleDrive,
        MotorConfig.BackLeftModuleTurn,
        EncoderConfig.BackLeftModule,
        "BL");

    private final SwerveModule backRight = new SwerveModule(
        MotorConfig.BackRightModuleDrive,
        MotorConfig.BackRightModuleTurn,
        EncoderConfig.BackRightModule,
        "BR");

    private AHRS gyro = new AHRS(SPI.Port.kMXP);
    private SwerveDriveOdometry odometer = new SwerveDriveOdometry(kDriveKinematics, getRotation2d(), getModulePositions());

    private DataLog datalog = DataLogManager.getLog();
    private DoubleLogEntry translationXOutputLog = new DoubleLogEntry(datalog, "/swerve/txout"); //Logs x translation state output
    private DoubleLogEntry translationYOutputLog = new DoubleLogEntry(datalog, "/swerve/tyout"); //Logs y translation state output
    private DoubleLogEntry rotationOutputLog = new DoubleLogEntry(datalog, "/swerve/rotout"); //Logs rotation state output
    private BooleanLogEntry controlOrientationLog = new BooleanLogEntry(datalog, "/swerve/orientation"); //Logs if robot is in FOD/ROD
    private StringLogEntry errors = new StringLogEntry(datalog, "/swerve/errors"); //Logs any hardware errors

    public boolean controlOrientationIsFOD;

    public SwerveSubsystem() {
        zeroHeading();
        controlOrientationIsFOD = true;
    }

    public void zeroHeading() {
        if (gyro.isCalibrating()){errors.append("gyro failed to calibrate before zero");}
        gyro.reset();
    }

    public double getHeading() { 
        return gyro.getYaw();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public void toggleOrientation(){
        //Toggle control orientation from FOD/ROD
        controlOrientationIsFOD = !controlOrientationIsFOD;
        controlOrientationLog.append(controlOrientationIsFOD);
    }

    public SwerveModuleState[] convertToModuleStates(double xTranslation, double yTranslation, double rotation) {
        //Takes axis input from joysticks and returns an array of swerve module states

        double x = yTranslation; //Intentional, x in swerve kinematics is y on the joystick
        double y = xTranslation;
        double r = rotation;

        //Map to speeds in meters/radians per second
        x *= kPhysicalMaxSpeed;
        y *= kPhysicalMaxSpeed;
        r *= kPhysicalMaxAngularSpeed;

        //Log speeds used to construct chassis speeds
        translationXOutputLog.append(x);
        translationYOutputLog.append(y);
        rotationOutputLog.append(r);

        //Construct Chassis Speeds
        ChassisSpeeds chassisSpeeds;
        if(controlOrientationIsFOD){
            //Field Oriented Drive
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, r, this.getRotation2d());
        } else {
            //Robot Oriented Drive
            chassisSpeeds = new ChassisSpeeds(x, y, r);
        }
        SmartDashboard.putString("chassis speeds",chassisSpeeds.toString());
        //Convert Chassis Speeds to individual module states
        return kDriveKinematics.toSwerveModuleStates(chassisSpeeds);  
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kPhysicalMaxSpeed);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] swerveModuleArray = new SwerveModuleState[4];
        swerveModuleArray[0] = frontLeft.getState();
        swerveModuleArray[1] = frontRight.getState();
        swerveModuleArray[2] = backLeft.getState();
        swerveModuleArray[3] = backRight.getState();

        return swerveModuleArray;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] swerveModuleArray = new SwerveModulePosition[4];
        swerveModuleArray[0] = frontLeft.getPosition();
        swerveModuleArray[1] = frontRight.getPosition();
        swerveModuleArray[2] = backLeft.getPosition();
        swerveModuleArray[3] = backRight.getPosition();

        return swerveModuleArray;
    }  

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getModulePositions());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }
}