package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.DriveConstants;
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

    public SwerveSubsystem() {
        new WaitUntilCommand(this::gyroReady)
        .andThen(new InstantCommand(this::zeroHeading,this))
        .schedule();
    }

    public boolean gyroReady() {
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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
    }

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeed);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

}