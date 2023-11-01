/*!
 * Copyright (c) FIRST and other WPILib contributors.
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 * 
 * @file SwerveModule.java
 *
 * @brief The main class for controlling Swerve Drives
 *
 * @section Changelog
 * Co-authored-by: JP Cassar <jp@cassartx.net>
 * Updated to add oxconfig
 */

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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
import frc.robot.classes.RelativeEncoderSim;
import frc.robot.hardware.AbsoluteEncoder;
import frc.robot.hardware.AbsoluteEncoder.EncoderConfig;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;
import me.nabdev.oxconfig.ConfigurableParameter;

public class SwerveModule extends SubsystemBase {
    public static final ConfigurableParameter<Double> kWheelDiameterMeters = new ConfigurableParameter<Double>(
        Units.inchesToMeters(3.5), 
        "wheel/diameter/meters"
    );
    
    public static final ConfigurableParameter<Double> kDriveMotorGearRatio = new ConfigurableParameter<Double>(
        (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0), 
        "drive/motor/gear/ratio"
    );
    
    public static final ConfigurableParameter<Double> kTurningMotorGearRatio = new ConfigurableParameter<Double>(
        (14.0 / 50.0) * (10.0 / 60.0), 
        "turning/motor/gear/ratio"
    );
    
    public static final ConfigurableParameter<Double> kDriveEncoderRotFactor = new ConfigurableParameter<Double>(
        kDriveMotorGearRatio.get() * Math.PI * kWheelDiameterMeters.get(), //Conversion factor converting the Drive Encoder's rotations to meters
        "drive/encoder/rot/factor"
    );
    
    public static final ConfigurableParameter<Double> kDriveEncoderRPMFactor = new ConfigurableParameter<Double>(
        kDriveEncoderRotFactor.get() / 60, //Conversion factor converting the Drive Encoder's RPM to meters per second
        "drive/encoder/rpm/factor"
    );
    
    public static final ConfigurableParameter<Double> kTurningEncoderRotFactor = new ConfigurableParameter<Double>(
        kTurningMotorGearRatio.get() * 2 * Math.PI, // Conversion factor converting the Turn Encoder's rotations to Radians
        "turning/encoder/rot/factor"
    );
    
    public static final ConfigurableParameter<Double> kTurningEncoderRPMFactor = new ConfigurableParameter<Double>(
        kTurningEncoderRotFactor.get() / 60, // Conversion factor converting the Turn Encoder's RPM to radians per second
        "turning/encoder/rpm/factor"
    );
    
    public static final ConfigurableParameter<Double> kPTurning = new ConfigurableParameter<Double>(
        0.4, //P gain for the turning motor
        "pturning"
    );

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final FlywheelSim simTurningMotor;
    private final FlywheelSim simDriveMotor;

    private final RelativeEncoderSim simDriveEncoder;
    private final RelativeEncoderSim simTurningEncoder;

    private final SparkMaxPIDController turningPIDController;
    private final PIDController simTurningPIDController;
    private double turningSetpoint;

    private final WPI_CANCoder absoluteEncoder;

    private DataLog datalog = DataLogManager.getLog();
    private DoubleLogEntry desiredSpeedLog;
    private DoubleLogEntry actualSpeedLog;
    private DoubleLogEntry desiredAngleLog;
    private DoubleLogEntry actualAngleLog;
    private DoubleLogEntry rotationSpeedLog;

    private final String ID;

    public SwerveModule(MotorConfig driveMotorConfig, MotorConfig turningMotorConfig, EncoderConfig absoluteEncoderConfig, String ID) {

        this.ID = ID;

        absoluteEncoder = Robot.isSimulation() ? null : AbsoluteEncoder.constructCANCoder(absoluteEncoderConfig);

        driveMotor = MotorController.constructMotor(driveMotorConfig);
        turningMotor = MotorController.constructMotor(turningMotorConfig);

        driveEncoder = this.driveMotor.getEncoder();
        turningEncoder = this.turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(kDriveEncoderRotFactor.get());
        driveEncoder.setVelocityConversionFactor(kDriveEncoderRPMFactor.get());
        turningEncoder.setPositionConversionFactor(kTurningEncoderRotFactor.get());
        turningEncoder.setVelocityConversionFactor(kTurningEncoderRPMFactor.get());

        simDriveMotor = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(2, 1.24), 
            DCMotor.getNEO(1),
            kDriveMotorGearRatio.get()
        );
        simTurningMotor = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(0.16, 0.0348), 
            DCMotor.getNEO(1),
            kTurningMotorGearRatio.get()
        );

        simDriveEncoder = new RelativeEncoderSim(driveMotor);
        simTurningEncoder = new RelativeEncoderSim(turningMotor);
        
        turningPIDController = turningMotor.getPIDController();
        
        turningPIDController.setP(kPTurning.get());

        simTurningPIDController = new PIDController(turningPIDController.getP(), turningPIDController.getI(), turningPIDController.getD());
        

        simTurningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();

        desiredSpeedLog = new DoubleLogEntry(datalog, "/swerve/" + ID +"/setSpeed"); //Logs desired speed in meters per second
        actualSpeedLog = new DoubleLogEntry(datalog, "/swerve/" + ID +"/setSpeed"); //Logs actual speed in meters per second
        desiredAngleLog = new DoubleLogEntry(datalog, "/swerve/" + ID +"/setAngle"); //Logs desired angle in radians
        actualAngleLog = new DoubleLogEntry(datalog, "/swerve/" + ID +"/actualAngle"); //Logs actual relative angle in radians
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRadians(getTurningPosition()));
    }

    public double getDrivePosition() {
        return Robot.isSimulation() ? simDriveEncoder.getPosition() : driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        //The math for this remainder is position - (2pi * Math.round(position/2pi))
        return Robot.isSimulation() ? Math.IEEEremainder(simTurningEncoder.getPosition(), Math.PI * 2) : Math.IEEEremainder(turningEncoder.getPosition(), Math.PI * 2);
    }

    public double getAbsoluteTurningPosition(){
        return Units.degreesToRadians(Robot.isSimulation() ? 0 : absoluteEncoder.getAbsolutePosition());
    }

    public double getDriveVelocity() {
        return Robot.isSimulation() ? simDriveEncoder.getVelocity() : driveEncoder.getVelocity();
    }
    
    public double getTurningVelocity() {
        double currentRotation = Robot.isSimulation() ? simTurningEncoder.getVelocity() : turningEncoder.getVelocity();
        rotationSpeedLog.append(currentRotation);
        return currentRotation;
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteTurningPosition());
        if(Robot.isReal()){
            absoluteEncoder.close();
        }
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    private double calculateSetpoint(double stateAngle){
        return turningEncoder.getPosition() + Math.IEEEremainder(stateAngle - getTurningPosition(), 2 * Math.PI); 
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) == 0) { //Prevent wheels to returning to heading of 0 when controls released
            stop();
            return;
        }
        
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        driveMotor.set(desiredState.speedMetersPerSecond / SwerveSubsystem.kPhysicalMaxSpeed);
        turningSetpoint = calculateSetpoint(desiredState.angle.getRadians());
        turningPIDController.setReference(turningSetpoint, ControlType.kPosition);

        SmartDashboard.putString("Swerve[" + ID + "] state", desiredState.toString());
        desiredAngleLog.append(desiredState.angle.getRadians());
        desiredSpeedLog.append(desiredState.speedMetersPerSecond);
    }

    public void park(boolean reversed) {
        stop();
        if(reversed){
            //not using calculateSetpoint because these are less than one full rotation
            turningPIDController.setReference(Math.PI/4, ControlType.kPosition);  
        } else{
            turningPIDController.setReference(-Math.PI/4, ControlType.kPosition);
        }
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public void coast() {
        driveMotor.setIdleMode(IdleMode.kCoast);
        turningMotor.setIdleMode(IdleMode.kCoast);
    }

    public void brake() {
        driveMotor.setIdleMode(IdleMode.kBrake);
        turningMotor.setIdleMode(IdleMode.kBrake);
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
        try {
            turningMotor.set(simTurningPIDController.calculate(getTurningPosition(), turningSetpoint));
        } catch (NullPointerException npe) {
            throw new NullPointerException(npe.getLocalizedMessage());
        }
  }

    @Override
    public void periodic(){
        actualSpeedLog.append(driveEncoder.getVelocity());
        actualAngleLog.append(getTurningPosition());
    }
}
