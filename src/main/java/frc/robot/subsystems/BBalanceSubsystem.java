// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

public class BBalanceSubsystem extends SubsystemBase { // TODO: Find a permanent name for buddy balance subsystem - BBalance is ambiguous
  private SparkMaxPIDController BB_PIDController;
  private CANSparkMax rightBBalanceMotor;
  private boolean toggle = false;

  public BBalanceSubsystem() {
    rightBBalanceMotor = MotorController.constructMotor(MotorConfig.BBalanceRightShaftRotate);
    BB_PIDController = rightBBalanceMotor.getPIDController();
    BB_PIDController.setP(1); // placeholder PID values
    BB_PIDController.setI(0);
    BB_PIDController.setD(0);
    BB_PIDController.setFF(0);
    BB_PIDController.setReference(0, CANSparkMax.ControlType.kPosition);
  }


  public void ToggleBuddyBalance(){ // changes the ref point for the buddy balance lift
    toggle ^= true;
    if (toggle) {
      BB_PIDController.setReference(50, CANSparkMax.ControlType.kPosition);
    } else if (toggle == false) {
      BB_PIDController.setReference(0, CANSparkMax.ControlType.kPosition);
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}