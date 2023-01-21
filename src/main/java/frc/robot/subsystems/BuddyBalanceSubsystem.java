// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

public class BuddyBalanceSubsystem extends SubsystemBase { // TODO: Find a permanent name for buddy balance subsystem - BBalance is ambiguous
  private SparkMaxPIDController rightPIDController;
  private SparkMaxPIDController leftPIDController;
  private CANSparkMax rightMotor;
  private CANSparkMax leftMotor;
  private boolean isToggled = false;

  public BuddyBalanceSubsystem() {
    rightMotor = MotorController.constructMotor(MotorConfig.BBalanceRightShaftRotate);
    leftMotor = MotorController.constructMotor(MotorConfig.BBalanceLeftShaftRotate);
    rightPIDController = rightMotor.getPIDController();
    leftPIDController = leftMotor.getPIDController();
    rightPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
    leftPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
  }

  public void deployBuddyBalance() { // changes the ref point for the buddy balance lift
    isToggled = !isToggled;
    if (isToggled) {
      rightPIDController.setReference(50, CANSparkMax.ControlType.kPosition);
      leftPIDController.setReference(50, CANSparkMax.ControlType.kPosition);
    } else {
      rightPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
      leftPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
    }
  }

  public void retrieveBuddy() {
    rightPIDController.setReference(15, CANSparkMax.ControlType.kPosition);
    leftPIDController.setReference(15, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}