// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.TunableNumber;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;

public class BuddyBalanceSubsystem extends SubsystemBase {
  private static double kBalancedPosition; // Buddy balance PID reference point when lifting a robot and engaging charge station
  private static double kDeployedPosition; // Buddy balance PID reference point when setting down a robot/initial position when deployed
  // Make the reference point constants and default motor PID values final when they are done being tuned with TunableNumbers
  private static final double kDefaultMotorP = 1e-6;
  private static final double kDefaultMotorI = 0;
  private static final double kDefaultMotorD = 1e-6;
  private static final int deployServoID = 1;
  private static double kServoDeployedPos = 1;

  private Servo activateDeploy;
  private CANSparkMax rightMotor;
  private CANSparkMax leftMotor;
  private SparkMaxPIDController rightPIDController;
  private SparkMaxPIDController leftPIDController;
  private RelativeEncoder rightEncoder;
  private RelativeEncoder leftEncoder;
  private boolean isDeployed = false;
  private DataLog datalog = DataLogManager.getLog();
  private DoubleLogEntry buddyBalancePosition = new DoubleLogEntry(datalog, "/buddybalance/position");
  private ShuffleboardTab buddyBalanceTab = Shuffleboard.getTab("Buddy Balance"); // Replace buddy balance tab with whatever tab the position should be logged to


  public BuddyBalanceSubsystem() {
    rightMotor = MotorController.constructMotor(MotorConfig.BuddyBalanceRight);
    leftMotor = MotorController.constructMotor(MotorConfig.BuddyBalanceLeft);
    activateDeploy = new Servo(deployServoID);
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

    new TunableNumber("Buddy Balance Right Motor P", kDefaultMotorP, rightPIDController::setP);
    new TunableNumber("Buddy Balance Right Motor I", kDefaultMotorI, rightPIDController::setI);
    new TunableNumber("Buddy Balance Right Motor D", kDefaultMotorD, rightPIDController::setD);
    new TunableNumber("Buddy Balance Left Motor P", kDefaultMotorP, leftPIDController::setP);
    new TunableNumber("Buddy Balance Left Motor I", kDefaultMotorI, leftPIDController::setI);
    new TunableNumber("Buddy Balance Left Motor D", kDefaultMotorD, leftPIDController::setD);

    new TunableNumber("Ref Point Balanced", 15, (a) -> {kBalancedPosition = a;});
    new TunableNumber("Ref Point Deployed", 0, (a) -> {kDeployedPosition = a;});
    new TunableNumber("Ref Point Servo", 1, (a) -> {kServoDeployedPos = a;});

    buddyBalanceTab.add("Buddy Balance Position", 0).getEntry();
  }

  public boolean getIsDeployed() {
    return isDeployed;
  }

  public void deployBuddyBalance() {
    activateDeploy.set(kServoDeployedPos);
    isDeployed = true;
  }

  public void retrieveBuddy() { // Used to pick up the buddy robot while the lift is already underneath it
    rightPIDController.setReference(kBalancedPosition, CANSparkMax.ControlType.kPosition);
    leftPIDController.setReference(kBalancedPosition, CANSparkMax.ControlType.kPosition);
  }

  public void releaseBuddy() { // Used to set down the robot
    rightPIDController.setReference(kDeployedPosition, CANSparkMax.ControlType.kPosition);
    leftPIDController.setReference(kDeployedPosition, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(this);
    // This method will be called once per scheduler run
    buddyBalancePosition.append(rightEncoder.getPosition()); // Logging the position of the buddy balance lift
    buddyBalancePosition.append(leftEncoder.getPosition());
  }
}