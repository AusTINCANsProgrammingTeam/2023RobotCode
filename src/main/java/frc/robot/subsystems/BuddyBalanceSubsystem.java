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

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class BuddyBalanceSubsystem extends SubsystemBase {
  // public static double kDockedPosition; // Docked position may or may not be used depending on final design of buddy balance
  public static double kBalancedPosition; // Buddy balance PID reference point when lifting a robot and engaging charge station
  public static double kDeployedPosition; // Buddy balance PID reference point when setting down a robot/initial position when deployed
  // TODO: Make the reference point constants final when they are done being tuned with TunableNumbers
  public static final double kDefaultMotorP = 0.000001;
  public static final double kDefaultMotorI = 0;
  public static final double kDefaultMotorD = 0.000001;
  private TunableNumber refPointDockedTuner;
  private TunableNumber refPointBalancedTuner;
  private TunableNumber refPointDeployedTuner;

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
  private boolean isDeployed = false;

  private TunableNumber tNumBuddyBalanceRightP;
  private TunableNumber tNumBuddyBalanceRightI;
  private TunableNumber tNumBuddyBalanceRightD;
  private TunableNumber tNumBuddyBalanceLeftP;
  private TunableNumber tNumBuddyBalanceLeftI;
  private TunableNumber tNumBuddyBalanceLeftD;

  public BuddyBalanceSubsystem() {
    rightMotor = MotorController.constructMotor(MotorConfig.BuddyBalanceRight);
    leftMotor = MotorController.constructMotor(MotorConfig.BuddyBalanceLeft);
    rightPIDController = rightMotor.getPIDController();
    leftPIDController = leftMotor.getPIDController();
    rightPIDController.setP(kDefaultMotorP);
    rightPIDController.setI(kDefaultMotorI);
    rightPIDController.setD(kDefaultMotorD);
    leftPIDController.setP(kDefaultMotorP);
    leftPIDController.setI(kDefaultMotorI);
    leftPIDController.setD(kDefaultMotorD);
    rightEncoder = rightMotor.getEncoder();
    leftEncoder = leftMotor.getEncoder();

    tNumBuddyBalanceRightP = new TunableNumber("Buddy Balance Right Motor P", kDefaultMotorP, rightPIDController::setP);
    tNumBuddyBalanceRightI = new TunableNumber("Buddy Balance Right Motor I", kDefaultMotorI, rightPIDController::setI);
    tNumBuddyBalanceRightD = new TunableNumber("Buddy Balance Right Motor D", kDefaultMotorD, rightPIDController::setD);
    tNumBuddyBalanceLeftP = new TunableNumber("Buddy Balance Left Motor P", kDefaultMotorP, leftPIDController::setP);
    tNumBuddyBalanceLeftI = new TunableNumber("Buddy Balance Left Motor I", kDefaultMotorI, leftPIDController::setI);
    tNumBuddyBalanceLeftD = new TunableNumber("Buddy Balance Left Motor D", kDefaultMotorD, leftPIDController::setD);

    // refPointDockedTuner = new TunableNumber("Ref Point Docked", 50, (a) -> {kDockedPosition = a;});
    refPointBalancedTuner = new TunableNumber("Ref Point Balanced", 15, (a) -> {kBalancedPosition = a;});
    refPointDeployedTuner = new TunableNumber("Ref Point Deployed", 0, (a) -> {kDeployedPosition = a;});

    positionEntry = buddyBalanceTab.add("Buddy Balance Position", 0).getEntry();
  }

  public void deployBuddyBalance() {
    if(!isDeployed) {
      // TODO: Find out what mechanism will deploy the buddy balance and program the deploying
      isDeployed = true;
    }
  }

  public void retrieveBuddy() { // Used to pick up the buddy robot while the lift is already underneath it
    if(isDeployed) {
      rightPIDController.setReference(kBalancedPosition, CANSparkMax.ControlType.kPosition);
      leftPIDController.setReference(kBalancedPosition, CANSparkMax.ControlType.kPosition);
    }
  }

  public void releaseBuddy() { // Used to set down the robot
    if(isDeployed) {
      rightPIDController.setReference(kDeployedPosition, CANSparkMax.ControlType.kPosition);
      leftPIDController.setReference(kDeployedPosition, CANSparkMax.ControlType.kPosition);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    buddyBalancePosition.append(rightEncoder.getPosition()); // Logging the position of the buddy balance lift
    buddyBalancePosition.append(leftEncoder.getPosition());
  }
}