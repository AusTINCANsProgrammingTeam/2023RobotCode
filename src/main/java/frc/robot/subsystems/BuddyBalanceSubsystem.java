// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;
import frc.robot.classes.TunableNumber;

import javax.management.relation.Relation;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class BuddyBalanceSubsystem extends SubsystemBase { // TODO: Find a permanent name for buddy balance subsystem - BBalance is ambiguous
  public static final int buddyBalanceRefPointDocked = 50;
  public static final int buddyBalanceRefPointBalanced = 15; // TODO: Replace constants with actual ref points during testing
  public static final int buddyBalanceRefPointDeployed = 0;

  private CANSparkMax rightMotor;
  private CANSparkMax leftMotor;
  private SparkMaxPIDController rightPIDController;
  private SparkMaxPIDController leftPIDController;
  private RelativeEncoder rightEncoder;
  private RelativeEncoder leftEncoder;
  private DataLog datalog = DataLogManager.getLog();
  private DoubleLogEntry buddyBalancePosition = new DoubleLogEntry(datalog, "/buddybalance/position");
  private ShuffleboardTab buddyBalanceTab = Shuffleboard.getTab("Buddy Balance"); // TODO: Replace buddy balance tab with whatever tab the position should be logged to
  private GenericEntry positionEntry;
  private boolean isToggled = false;
  private TunableNumber tNumBuddyBalanceRightP;
  private TunableNumber tNumBuddyBalanceRightI;
  private TunableNumber tNumBuddyBalanceRightD;
  private TunableNumber tNumBuddyBalanceLeftP;
  private TunableNumber tNumBuddyBalanceLeftI;
  private TunableNumber tNumBuddyBalanceLeftD;

  public BuddyBalanceSubsystem() {
    rightMotor = MotorController.constructMotor(MotorConfig.BBalanceRightShaftRotate);
    leftMotor = MotorController.constructMotor(MotorConfig.BBalanceLeftShaftRotate);
    rightPIDController = rightMotor.getPIDController();
    leftPIDController = leftMotor.getPIDController();
    rightEncoder = rightMotor.getEncoder();
    leftEncoder = leftMotor.getEncoder();
    rightPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
    leftPIDController.setReference(0, CANSparkMax.ControlType.kPosition);

    tNumBuddyBalanceRightP = new TunableNumber("Buddy Balance Right Motor P", 0, rightPIDController::setP);
    tNumBuddyBalanceRightI = new TunableNumber("Buddy Balance Right Motor I", 0, rightPIDController::setI);
    tNumBuddyBalanceRightD = new TunableNumber("Buddy Balance Right Motor D", 0, rightPIDController::setD);
    tNumBuddyBalanceLeftP = new TunableNumber("Buddy Balance Left Motor P", 0, leftPIDController::setP);
    tNumBuddyBalanceLeftI = new TunableNumber("Buddy Balance Left Motor I", 0, leftPIDController::setI);
    tNumBuddyBalanceLeftD = new TunableNumber("Buddy Balance Left Motor D", 0, leftPIDController::setD);

    positionEntry = buddyBalanceTab.add("Buddy Balance Position", 0).getEntry();
  }

  public void deployBuddyBalance() { // changes the ref point for the buddy balance lift
    isToggled = !isToggled;
    if (isToggled) {
      rightPIDController.setReference(buddyBalanceRefPointDocked, CANSparkMax.ControlType.kPosition);
      leftPIDController.setReference(buddyBalanceRefPointDocked, CANSparkMax.ControlType.kPosition);
    } else {
      rightPIDController.setReference(buddyBalanceRefPointDeployed, CANSparkMax.ControlType.kPosition);
      leftPIDController.setReference(buddyBalanceRefPointDeployed, CANSparkMax.ControlType.kPosition);
    }
  }

  public void retrieveBuddy() {
    rightPIDController.setReference(buddyBalanceRefPointBalanced, CANSparkMax.ControlType.kPosition);
    leftPIDController.setReference(buddyBalanceRefPointBalanced, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    buddyBalancePosition.append(rightEncoder.getPosition()); // Logging the position of the buddy balance lift
  }
}