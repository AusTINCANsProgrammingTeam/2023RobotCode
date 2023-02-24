// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
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
  public static double kRetrievedAngle; // Buddy balance PID reference point when lifting a robot and engaging charge station
  public static double kDeployedAngle; // Buddy balance PID reference point when setting down a robot/initial position when deployed
  // TODO: Make the constants final when they are done being tuned with TunableNumbers
  private static final double kDefaultMotorP = 1e-6;
  private static final double kDefaultMotorI = 0;
  private static final double kDefaultMotorD = 1e-6;
  private static final int deployServoID = 1;
  public static double kServoDeployedPos; // public because of JUnit
  public static double rightPIDControllerEncoderAngle; 
  public static double leftPIDControllerEncoderAngle;

  private TunableNumber refPointBalancedTuner;
  private TunableNumber refPointDeployedTuner;
  private TunableNumber refPointServoTuner;
  private TunableNumber tunerNumRightP;
  private TunableNumber tunerNumRightI;
  private TunableNumber tunerNumRightD;
  private TunableNumber tunerNumLeftP;
  private TunableNumber tunerNumLeftI;
  private TunableNumber tunerNumLeftD;

  private Servo activateDeploy; // public because of JUnit
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
  private static ShuffleboardTab buddyBalanceTab = Shuffleboard.getTab("Buddy Balance"); // TODO: Replace buddy balance tab with whatever tab the position should be logged to
  private static GenericEntry positionEntry = buddyBalanceTab.add("Buddy Balance Position", 0).getEntry();
  //private static GenericEntry buddyBalancePosEntry = SwerveSubsystem.matchTab.add("Buddy Balance State", "Docked").getEntry();

  public BuddyBalanceSubsystem() {
    rightMotor = MotorController.constructMotor(MotorConfig.BuddyBalanceRight);
    leftMotor = MotorController.constructMotor(MotorConfig.BuddyBalanceLeft);
    activateDeploy = new Servo(deployServoID);
    rightEncoder = AbsoluteEncoder.constructREVEncoder(EncoderConfig.BuddyBalanceRight);
    leftEncoder = AbsoluteEncoder.constructREVEncoder(EncoderConfig.BuddyBalanceLeft);
    rightPIDController = new PIDController(kDefaultMotorP, kDefaultMotorI, kDefaultMotorD);
    leftPIDController = new PIDController(kDefaultMotorP, kDefaultMotorI, kDefaultMotorD);
    rightPIDControllerEncoderAngle = rightPIDController.calculate(getRightAngle());
    leftPIDControllerEncoderAngle = leftPIDController.calculate(getLeftAngle());

    tunerNumRightP = new TunableNumber("Buddy Balance Right Motor P", kDefaultMotorP, rightPIDController::setP);
    tunerNumRightI = new TunableNumber("Buddy Balance Right Motor I", kDefaultMotorI, rightPIDController::setI);
    tunerNumRightD = new TunableNumber("Buddy Balance Right Motor D", kDefaultMotorD, rightPIDController::setD);
    tunerNumLeftP = new TunableNumber("Buddy Balance Left Motor P", kDefaultMotorP, leftPIDController::setP);
    tunerNumLeftI = new TunableNumber("Buddy Balance Left Motor I", kDefaultMotorI, leftPIDController::setI);
    tunerNumLeftD = new TunableNumber("Buddy Balance Left Motor D", kDefaultMotorD, leftPIDController::setD);

    refPointBalancedTuner = new TunableNumber("Ref Point Balanced", 0, (a) -> {kRetrievedAngle = a;});
    refPointDeployedTuner = new TunableNumber("Ref Point Deployed", 0, (a) -> {kDeployedAngle = a;});
    refPointServoTuner = new TunableNumber("Ref Point Servo", 0, (a) -> {kServoDeployedPos = a;});
  }

  public double getRightAngle() {
    return Math.round(AbsoluteEncoder.getPositionRadians(rightEncoder));
  }

  public double getLeftAngle() {
    return Math.round(AbsoluteEncoder.getPositionRadians(leftEncoder));
  }

  public boolean getIsDeployed() {
    return isDeployed;
  }

  public void deployBuddyBalance() {
    activateDeploy.set(kServoDeployedPos);
    isDeployed = true;
  }

  public void retrieveBuddy() { // Used to pick up the buddy robot while the lift is already underneath it
    rightPIDController.setSetpoint(kRetrievedAngle);
    leftPIDController.setSetpoint(kRetrievedAngle);
    //buddyBalancePosEntry.setString("Raised");
  }

  public void releaseBuddy() { // Used to set down the robot
    rightPIDController.setSetpoint(kDeployedAngle);
    leftPIDController.setSetpoint(kDeployedAngle);
    //buddyBalancePosEntry.setString("Lowered");
  }

  public void updateMotors() {
    rightMotor.set(MathUtil.clamp(rightPIDControllerEncoderAngle, -1, 1));
    leftMotor.set(MathUtil.clamp(leftPIDControllerEncoderAngle, -1, 1));
  }

  // These 4 methods are used for JUnit testing only
  public void close() throws Exception {
    // This method will close all device handles used by this object and release any other dynamic memory.
    // Mostly for JUnit tests
    rightMotor.close();
    leftMotor.close();
    activateDeploy.close();
    rightEncoder.close();
    leftEncoder.close();
  }

  public double getServoAngle() {
    return activateDeploy.getAngle();
  }

  public double getEncoderRightAngle() {
    return rightEncoder.getAbsolutePosition();
  }

  public double getEncoderLeftAngle() {
    return leftEncoder.getAbsolutePosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateMotors();
    buddyBalancePosLeft.append(AbsoluteEncoder.getPositionRadians(leftEncoder)); // Logging the position of the buddy balance lift
    buddyBalancePosRight.append(AbsoluteEncoder.getPositionRadians(rightEncoder));
    SmartDashboard.putNumber("Buddy Balance Right Position", AbsoluteEncoder.getPositionRadians(rightEncoder));
    SmartDashboard.putNumber("Buddy Balance Left Position", AbsoluteEncoder.getPositionRadians(leftEncoder));
  }
}