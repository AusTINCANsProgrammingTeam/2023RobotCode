// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.classes.RelativeEncoderSim;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class SwerveModule extends SubsystemBase {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoderSim simDriveEncoder;
    private final RelativeEncoderSim simTurningEncoder;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPIDController;

    private double simTurningEncoderDistance;
    private double simDriveEncoderDistance;

    private double driveOutput = 0;
    private double turnOutput = 0;

    //TODO: FIX THE NUMBERS!
    private final FlywheelSim moduleRotationSimModel = new FlywheelSim(
                  LinearSystemId.identifyVelocitySystem(0.16, 0.0348),
                  DCMotor.getNEO(1),
                  12.8
          );
        
          private final FlywheelSim moduleThrottleSimModel = new FlywheelSim(
                  LinearSystemId.identifyVelocitySystem(2, 1.24),
                  DCMotor.getNEO(1),
                  8.16
          );

    private final Supplier<Double> encoderSupplier;

    private final String ID;

    public SwerveModule(CANSparkMax driveMotor, CANSparkMax turningMotor, Supplier<Double> encoderSupplier, String ID) {

        this.ID = ID;

        this.encoderSupplier = encoderSupplier;

        this.driveMotor = driveMotor;
        this.turningMotor = turningMotor;

        driveEncoder = this.driveMotor.getEncoder();
        turningEncoder = this.turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(SwerveModuleConstants.kDriveEncoderRotFactor);
        driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDriveEncoderRPMFactor);
        turningEncoder.setPositionConversionFactor(SwerveModuleConstants.kTurningEncoderRotFactor);
        turningEncoder.setVelocityConversionFactor(SwerveModuleConstants.kTurningEncoderRPMFactor);

        turningPIDController = new PIDController(SwerveModuleConstants.kPTurning, 0, 0);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        simDriveEncoder = new RelativeEncoderSim(driveMotor);
        simTurningEncoder = new RelativeEncoderSim(turningMotor);

        simTurningEncoderDistance = 0;
        simDriveEncoderDistance = 0;

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return Robot.isSimulation() ? simDriveEncoder.getVelocity() : driveEncoder.getVelocity();
    }
    

    public double getTurningVelocity() {
        return Robot.isSimulation() ? simTurningEncoder.getVelocity() : turningEncoder.getVelocity();
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(Robot.isSimulation() ? 0 : encoderSupplier.get());
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
        turnOutput = turningPIDController.calculate(getTurningPosition(), state.angle.getRadians());
        driveOutput = state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeed;
        driveMotor.set(driveOutput);
        turningMotor.set(turnOutput);
        SmartDashboard.putString("Swerve[" + ID + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
        turnOutput = 0;
        driveOutput = 0;
    }

    @Override
    public void simulationPeriodic() {
        
        moduleRotationSimModel.setInputVoltage(turnOutput  * RobotController.getBatteryVoltage());
        moduleThrottleSimModel.setInputVoltage(driveOutput * RobotController.getBatteryVoltage());

        moduleRotationSimModel.update(Robot.kDefaultPeriod);
        moduleThrottleSimModel.update(Robot.kDefaultPeriod);

        if (Math.abs(moduleRotationSimModel.getAngularVelocityRadPerSec()) >= Constants.kSimVelocityDeadzone) {
            simTurningEncoderDistance += moduleRotationSimModel.getAngularVelocityRadPerSec() * Robot.kDefaultPeriod;
        }
        simTurningEncoder.setPosition(simTurningEncoderDistance);
        simTurningEncoder.setSimVelocity(moduleRotationSimModel.getAngularVelocityRadPerSec());

        if (Math.abs(moduleThrottleSimModel.getAngularVelocityRadPerSec()) >= Constants.kSimVelocityDeadzone) {
            simDriveEncoderDistance += moduleThrottleSimModel.getAngularVelocityRadPerSec() * Robot.kDefaultPeriod;
        }
        simDriveEncoder.setPosition(simDriveEncoderDistance);
        simDriveEncoder.setSimVelocity(moduleThrottleSimModel.getAngularVelocityRadPerSec());
  }
}
