// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
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

public class SwerveModule extends SubsystemBase {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final FlywheelSim simTurningMotor;
    private final FlywheelSim simDriveMotor;

    private final RelativeEncoderSim simDriveEncoder;
    private final RelativeEncoderSim simTurningEncoder;

    private final PIDController turningPIDController;

    private final WPI_CANCoder absoluteEncoder;

    private final String ID;

    private DataLog SwerveModuledatalog = DataLogManager.getLog();
    private DoubleLogEntry speedLog;
    private DoubleLogEntry rotationLog;
    private DoubleLogEntry setSpeedLog;
    private DoubleLogEntry setRotationLog;

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
        
        turningPIDController = new PIDController(SwerveModuleConstants.kPTurning, 0, 0);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();

        speedLog = new DoubleLogEntry(SwerveModuledatalog, "/swerveMod/" + ID + "/speed");
        rotationLog = new DoubleLogEntry(SwerveModuledatalog, "/swerveMod/" + ID + "/rotation");
        setSpeedLog = new DoubleLogEntry(SwerveModuledatalog, "/swerveMod/" + ID + "/setSpeed");
        setRotationLog = new DoubleLogEntry(SwerveModuledatalog, "/swerveMod/" + ID + "/setRotation");
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        double currentSpeed = Robot.isSimulation() ? simTurningEncoder.getVelocity() : turningEncoder.getVelocity();
        speedLog.append(currentSpeed);
        return currentSpeed;
    }
    

    public double getTurningVelocity() {
        double currentRotation = Robot.isSimulation() ? simTurningEncoder.getVelocity() : turningEncoder.getVelocity();
        rotationLog.append(currentRotation);
        return currentRotation;
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(Robot.isSimulation() ? 0 : Units.degreesToRotations(absoluteEncoder.getAbsolutePosition()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) { //Prevent wheels to returning to heading of 0 when controls released
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        double setSpeed = state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeed;
        driveMotor.set(setSpeed);
        setSpeedLog.append(setSpeed);

        double setRotation = turningPIDController.calculate(getTurningPosition(), state.angle.getRadians());
        turningMotor.set(setRotation);
        setRotationLog.append(setRotation);
        // SmartDashboard.putString("Swerve[" + ID + "] state", state.toString());
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
}
