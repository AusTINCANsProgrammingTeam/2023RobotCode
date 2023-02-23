// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.AbsoluteEncoder;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.AbsoluteEncoder.EncoderConfig;
import frc.robot.hardware.MotorController.MotorConfig;
import frc.robot.classes.TunableNumber;

import com.revrobotics.CANSparkMax;

public class BuddyBalanceSubsystem extends SubsystemBase {
  private static double kBalancedAngle; // Buddy balance PID reference point when lifting a robot and engaging charge station
  private static double kDeployedAngle; // Buddy balance PID reference point when setting down a robot/initial position when deployed
  // TODO: Make the reference point constants and default motor PID values final when they are done being tuned with TunableNumbers
  private static final double kDefaultMotorP = 1e-6;
  private static final double kDefaultMotorI = 0;
  private static final double kDefaultMotorD = 1e-6;
  private static final int deployServoID = 1;
  private static double kServoDeployedPos;

  private TunableNumber refPointBalancedTuner;
  private TunableNumber refPointDeployedTuner;
  private TunableNumber refPointServoTuner;
  private TunableNumber tunerNumRightP;
  private TunableNumber tunerNumRightI;
  private TunableNumber tunerNumRightD;
  private TunableNumber tunerNumLeftP;
  private TunableNumber tunerNumLeftI;
  private TunableNumber tunerNumLeftD;

  private Servo activateDeploy;
  private CANSparkMax rightMotor;
  private CANSparkMax leftMotor;
  private PIDController rightPIDController;
  private PIDController leftPIDController;
  private DutyCycleEncoder rightEncoder;
  private DutyCycleEncoder leftEncoder;
  private boolean isDeployed = false;
  private DataLog datalog = DataLogManager.getLog();
  private DoubleLogEntry buddyBalancePosLeft = new DoubleLogEntry(datalog, "/buddybalance/position/left");
  private DoubleLogEntry buddyBalancePosRight = new DoubleLogEntry(datalog, "/buddybalance/position/right");
  private ShuffleboardTab buddyBalanceTab = Shuffleboard.getTab("Buddy Balance"); // TODO: Replace buddy balance tab with whatever tab the position should be logged to
  private GenericEntry positionEntry;
  private GenericEntry buddyBalancePosEntry;

  public BuddyBalanceSubsystem() {
    rightMotor = MotorController.constructMotor(MotorConfig.BuddyBalanceRight);
    leftMotor = MotorController.constructMotor(MotorConfig.BuddyBalanceLeft);
    activateDeploy = new Servo(deployServoID);
    rightPIDController = new PIDController(kDefaultMotorP, kDefaultMotorI, kDefaultMotorD);
    leftPIDController = new PIDController(kDefaultMotorP, kDefaultMotorI, kDefaultMotorD);
    rightEncoder = AbsoluteEncoder.constructREVEncoder(EncoderConfig.BuddyBalanceRight);
    leftEncoder = AbsoluteEncoder.constructREVEncoder(EncoderConfig.BuddyBalanceLeft);

    tunerNumRightP = new TunableNumber("Buddy Balance Right Motor P", kDefaultMotorP, rightPIDController::setP);
    tunerNumRightI = new TunableNumber("Buddy Balance Right Motor I", kDefaultMotorI, rightPIDController::setI);
    tunerNumRightD = new TunableNumber("Buddy Balance Right Motor D", kDefaultMotorD, rightPIDController::setD);
    tunerNumLeftP = new TunableNumber("Buddy Balance Left Motor P", kDefaultMotorP, leftPIDController::setP);
    tunerNumLeftI = new TunableNumber("Buddy Balance Left Motor I", kDefaultMotorI, leftPIDController::setI);
    tunerNumLeftD = new TunableNumber("Buddy Balance Left Motor D", kDefaultMotorD, leftPIDController::setD);

    refPointBalancedTuner = new TunableNumber("Ref Point Balanced", 0, (a) -> {kBalancedAngle = a;});
    refPointDeployedTuner = new TunableNumber("Ref Point Deployed", 0, (a) -> {kDeployedAngle = a;});
    refPointServoTuner = new TunableNumber("Ref Point Servo", 0, (a) -> {kServoDeployedPos = a;});

    positionEntry = buddyBalanceTab.add("Buddy Balance Position", 0).getEntry();
    //buddyBalancePosEntry = SwerveSubsystem.matchTab.add("Buddy Balance State", "Docked").getEntry();
  }

  public boolean getIsDeployed() {
    return isDeployed;
  }

  public void deployBuddyBalance() {
    activateDeploy.set(kServoDeployedPos);
    isDeployed = true;
  }

  public void retrieveBuddy() { // Used to pick up the buddy robot while the lift is already underneath it
    rightPIDController.setSetpoint(kBalancedAngle);
    leftPIDController.setSetpoint(kBalancedAngle);
    //buddyBalancePosEntry.setString("Raised");
  }

  public void releaseBuddy() { // Used to set down the robot
    rightPIDController.setSetpoint(kDeployedAngle);
    leftPIDController.setSetpoint(kDeployedAngle);
    //buddyBalancePosEntry.setString("Lowered");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    buddyBalancePosLeft.append(AbsoluteEncoder.getPositionRadians(leftEncoder)); // Logging the position of the buddy balance lift
    buddyBalancePosRight.append(AbsoluteEncoder.getPositionRadians(rightEncoder));
    SmartDashboard.putNumber("Buddy Balance Right Position", AbsoluteEncoder.getPositionRadians(rightEncoder));
    SmartDashboard.putNumber("Buddy Balance Left Position", AbsoluteEncoder.getPositionRadians(leftEncoder));
  }
}