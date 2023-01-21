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
  private SparkMaxPIDController PIDController;
  private CANSparkMax rightMotor;
  private boolean isToggled = false;

  public BuddyBalanceSubsystem() {
    rightMotor = MotorController.constructMotor(MotorConfig.BBalanceRightShaftRotate);
    PIDController = rightMotor.getPIDController();
    PIDController.setReference(0, CANSparkMax.ControlType.kPosition);
  }


  public void toggleBuddyBalance() { // changes the ref point for the buddy balance lift
    isToggled = !isToggled;
    if (isToggled) {
      PIDController.setReference(50, CANSparkMax.ControlType.kPosition);
    } else {
      PIDController.setReference(0, CANSparkMax.ControlType.kPosition);
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}