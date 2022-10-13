// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.hardware.AbsoluteEncoders;
import frc.robot.hardware.MotorControllers;

public class SwerveSubsystem extends SubsystemBase{
    private final SwerveModule frontLeft = new SwerveModule(
        MotorControllers.FrontLeftModuleDrive.getMotor(),
        MotorControllers.FrontLeftModuleTurn.getMotor(),
        AbsoluteEncoders.FrontLeftModule.getAngleSupplier(),
        "FL");

    private final SwerveModule frontRight = new SwerveModule(
        MotorControllers.FrontRightModuleDrive.getMotor(),
        MotorControllers.FrontRightModuleTurn.getMotor(),
        AbsoluteEncoders.FrontRightModule.getAngleSupplier(),
        "FR");

    private final SwerveModule backLeft = new SwerveModule(
        MotorControllers.BackLeftModuleDrive.getMotor(),
        MotorControllers.BackLeftModuleTurn.getMotor(),
        AbsoluteEncoders.BackLeftModule.getAngleSupplier(),
        "BL");

    private final SwerveModule backRight = new SwerveModule(
        MotorControllers.BackRightModuleDrive.getMotor(),
        MotorControllers.BackRightModuleTurn.getMotor(),
        AbsoluteEncoders.BackRightModule.getAngleSupplier(),
        "BR");

    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    private DataLog datalog = DataLogManager.getLog();
    private DoubleLogEntry translationXOutputLog = new DoubleLogEntry(datalog, "/swerve/txout"); //Logs x translation state output
    private DoubleLogEntry translationYOutputLog = new DoubleLogEntry(datalog, "/swerve/tyout"); //Logs y translation state output
    private DoubleLogEntry rotationOutputLog = new DoubleLogEntry(datalog, "/swerve/rotout"); //Logs rotation state output
    private BooleanLogEntry controlOrientationLog = new BooleanLogEntry(datalog, "/swerve/orientation"); //Logs if robot is in FOD/ROD

    public boolean controlOrientationIsFOD;

    public SwerveSubsystem() {
        new WaitUntilCommand(this::gyroReady)
        .andThen(new InstantCommand(this::zeroHeading,this))
        .schedule();
        controlOrientationIsFOD = true;
    }

    private boolean gyroReady() {
        return !gyro.isCalibrating();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360); //Clamps angle output between -180 and 180 degrees
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
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
        x *= DriveConstants.kPhysicalMaxSpeed;
        y *= DriveConstants.kPhysicalMaxSpeed;
        r *= DriveConstants.kPhysicalMaxAngularSpeed;

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

        //Convert Chassis Speeds to individual module states
        return DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);  
    }

    private void normalizeDrive(SwerveModuleState[] desiredStates, ChassisSpeeds speeds){
        //Find magnitude of translation input, map to a scale of 1 in order to be comparable to rotation
        double translationalK = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) / DriveConstants.kPhysicalMaxSpeed;
        //Find magnitude of rotation input, map to a scale of 1 in order to be comparable to translation
        double rotationalK = Math.abs(speeds.omegaRadiansPerSecond) / DriveConstants.kPhysicalMaxAngularSpeed;
        //Use the larger of the two magnitudes for scaling
        double k = Math.max(translationalK, rotationalK);
      
        //Find the how fast the fastest spinning drive motor is spinning                                       
        double realMaxSpeed = 0.0;
        for (SwerveModuleState moduleState : desiredStates) {
          realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
        }
      
        //Map input magnitude back to speed in meters per second, divide by real speed to create scale
        double scale = Math.min(k * SwerveModuleConstants.kPhysicalMaxSpeed / realMaxSpeed, 1);
        //Desaturate speeds using that scale
        for (SwerveModuleState moduleState : desiredStates) {
          moduleState.speedMetersPerSecond *= scale;
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        /*desatureWheelSpeeds fully saturates a module in many combinations of rotation and translation input, 
        while this custom normalization avoids fully saturating a module to make the drive more controllable*/
        this.normalizeDrive(desiredStates, DriveConstants.kDriveKinematics.toChassisSpeeds(desiredStates));
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

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
    }
}