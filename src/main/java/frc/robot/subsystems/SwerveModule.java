// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.classes.RelativeEncoderSim;
import frc.robot.hardware.AbsoluteEncoder;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.AbsoluteEncoder.EncoderConfig;
import frc.robot.hardware.MotorController.MotorConfig;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class SwerveModule extends SubsystemBase {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final FlywheelSim simTurningMotor;
    private final FlywheelSim simDriveMotor;

    private final RelativeEncoderSim simDriveEncoder;
    private final RelativeEncoderSim simTurningEncoder;

    private final SparkMaxPIDController turningPIDController;

    private final WPI_CANCoder absoluteEncoder;

    private DataLog datalog = DataLogManager.getLog();
    private DoubleLogEntry desiredSpeedLog;
    private DoubleLogEntry actualSpeedLog;
    private DoubleLogEntry desiredAngleLog;
    private DoubleLogEntry actualAbsoluteAngleLog;
    private DoubleLogEntry actualRelativeAngleLog;

    private final String ID;

    public SwerveModule(MotorConfig driveMotorConfig, MotorConfig turningMotorConfig, EncoderConfig absoluteEncoderConfig, String ID) {

        this.ID = ID;

        absoluteEncoder = Robot.isSimulation() ? null : AbsoluteEncoder.constructEncoder(absoluteEncoderConfig);

        driveMotor = MotorController.constructMotor(driveMotorConfig);
        turningMotor = MotorController.constructMotor(turningMotorConfig);

        driveEncoder = this.driveMotor.getEncoder();
        turningEncoder = this.turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(SwerveModuleConstants.kDriveEncoderRotFactor);
        driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDriveEncoderRPMFactor);
        turningEncoder.setPositionConversionFactor(SwerveModuleConstants.kTurningEncoderRotFactor);
        turningEncoder.setVelocityConversionFactor(SwerveModuleConstants.kTurningEncoderRPMFactor);

        simDriveMotor = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(2, 1.24), //TODO: Update with real SysID
            DCMotor.getNEO(1),
            SwerveModuleConstants.kDriveMotorGearRatio
        );
        simTurningMotor = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(0.16, 0.0348), //TODO: Update with real SysID
            DCMotor.getNEO(1),
            SwerveModuleConstants.kTurningMotorGearRatio
        );

        simDriveEncoder = new RelativeEncoderSim(driveMotor);
        simTurningEncoder = new RelativeEncoderSim(turningMotor);
        
        turningPIDController = turningMotor.getPIDController();
        turningPIDController.setP(SwerveModuleConstants.kPTurning);

        resetEncoders();

        desiredSpeedLog = new DoubleLogEntry(datalog, "/swerve/" + ID +"/setSpeed"); //Logs desired speed in meters per second
        actualSpeedLog = new DoubleLogEntry(datalog, "/swerve/" + ID +"/setSpeed"); //Logs actual speed in meters per second
        desiredAngleLog = new DoubleLogEntry(datalog, "/swerve/" + ID +"/setAngle"); //Logs desired angle in radians
        actualAbsoluteAngleLog = new DoubleLogEntry(datalog, "/swerve/" + ID +"/actualAbsAngle"); //Logs actual absolute angle in radians
        actualRelativeAngleLog = new DoubleLogEntry(datalog, "/swerve/" + ID +"/actualRelAngle"); //Logs actual relative angle in radians
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        //The math for this remainder is position - (2pi * Math.round(position/2pi))
        return Math.IEEEremainder(turningEncoder.getPosition(), Math.PI * 2);
    }

    public double getAbsoluteTurningPosition(){
        return Units.degreesToRadians(Robot.isSimulation() ? 0 : absoluteEncoder.getAbsolutePosition());
    }

    public double getDriveVelocity() {
        return Robot.isSimulation() ? simDriveEncoder.getVelocity() : driveEncoder.getVelocity();
    }
    

    public double getTurningVelocity() {
        return Robot.isSimulation() ? simTurningEncoder.getVelocity() : turningEncoder.getVelocity();
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteTurningPosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    private double calculateSetpoint(double stateAngle){
        double error = stateAngle - getTurningPosition();
        if(Math.abs(error) > Math.PI){
            error = Math.IEEEremainder(error, 2 * Math.PI);
        }
        return turningEncoder.getPosition() + error; 
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) == 0) { //Prevent wheels to returning to heading of 0 when controls released
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeed);
        turningPIDController.setReference(calculateSetpoint(state.angle.getRadians()), ControlType.kPosition);

        SmartDashboard.putString("Swerve[" + ID + "] state", state.toString());
        desiredAngleLog.append(state.angle.getRadians());
        desiredSpeedLog.append(state.speedMetersPerSecond);
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    @Override
    public void simulationPeriodic() {
        simDriveMotor.setInputVoltage(driveMotor.get() * RobotController.getBatteryVoltage());
        simTurningMotor.setInputVoltage(turningMotor.get()  * RobotController.getBatteryVoltage());

        simDriveMotor.update(Robot.kDefaultPeriod);
        simTurningMotor.update(Robot.kDefaultPeriod);

        simDriveEncoder.setPosition(simDriveEncoder.getPosition() + simDriveMotor.getAngularVelocityRadPerSec() * Robot.kDefaultPeriod);
        simDriveEncoder.setSimVelocity(simDriveMotor.getAngularVelocityRadPerSec());

        simTurningEncoder.setPosition(simTurningEncoder.getPosition() + simTurningMotor.getAngularVelocityRadPerSec() * Robot.kDefaultPeriod);
        simTurningEncoder.setSimVelocity(simTurningMotor.getAngularVelocityRadPerSec());
  }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Swerve[" + ID + "] absolute encoder position", getAbsoluteTurningPosition());
        SmartDashboard.putNumber("Swerve[" + ID + "] relative encoder position", getTurningPosition());
        actualSpeedLog.append(driveEncoder.getVelocity());
        actualAbsoluteAngleLog.append(getAbsoluteTurningPosition());
        actualRelativeAngleLog.append(getTurningPosition());
    }
}
