// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
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
  private static final double kRetrievedAngle = 180; // Buddy balance PID reference point when lifting a robot and engaging charge station
  private static final double kDeployedAngle = 0; // Buddy balance PID reference point when setting down a robot/initial position when deployed
  private static final double kDefaultMotorP = 0.0001;
  private static final double kDefaultMotorI = 0;
  private static final double kDefaultMotorD = 0;
  private static final int deployServo1ID = 8;
  private static final int deployServo2ID = 9;
  private static final double kServoInitialPos = 0.5;
  private static final double kServoDeployedPos1 = 0;
  private static final double kServoDeployedPos2 = 1;
  private static final Constraints kConstraints = new Constraints(Units.degreesToRadians(180), Units.degreesToRadians(0));
  private double tunedRetrievedAngle;
  private double tunedDeployedAngle;
  private double tunedServoDeployedPos1;
  private double tunedServoDeployedPos2;

  private TunableNumber refPointBalancedTuner;
  private TunableNumber refPointDeployedTuner;
  private TunableNumber refPointServoTuner1;
  private TunableNumber refPointServoTuner2;
  private TunableNumber tunerNumP;
  private TunableNumber tunerNumI;
  private TunableNumber tunerNumD;

  private Servo deployServo1;
  private Servo deployServo2;
  private CANSparkMax rightMotor1;
  private CANSparkMax rightMotor2;
  private CANSparkMax leftMotor1;
  private CANSparkMax leftMotor2;
  private ProfiledPIDController unifiedPIDController; // Never forget
  private DutyCycleEncoder encoder;
  private boolean isDeployed = false;
  private DataLog datalog = DataLogManager.getLog();
  private DoubleLogEntry buddyBalancePos = new DoubleLogEntry(datalog, "/buddybalance/position/");
  private static ShuffleboardTab buddyBalanceTab = Shuffleboard.getTab("Buddy Balance"); // TODO: Replace buddy balance tab with whatever tab the position should be logged to
  private static GenericEntry buddyBalancePosEntry = buddyBalanceTab.add("Buddy Balance State", "Docked").getEntry();
  private static GenericEntry positionEntry = buddyBalanceTab.add("Buddy Balance Position", 0).getEntry();
  private GenericEntry rightMotorOutputEntry = buddyBalanceTab.add("Buddy Balance Right Motor Speed", 0).getEntry();// TODO: move to match tab when pull request goes through
  private GenericEntry leftMotorOutputEntry = buddyBalanceTab.add("Buddy Balance Left Motor Speed", 0).getEntry();
  private GenericEntry setpointEntry = buddyBalanceTab.add("Buddy Balance PID Setpoint", 0).getEntry();
  private GenericEntry servoPositionEntry = buddyBalanceTab.add("Buddy Balance Servo Position", 0).getEntry();
  // TODO: Uncomment comp shuffleboard objects when both comp-shuffleboard and buddy-balance-encoder pull requests go through

  private static double encoderCalculatedAngle; 

  public static final Constraints kBalanceConstraints = new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(360));

  public BuddyBalanceSubsystem() {
    rightMotor1 = MotorController.constructMotor(MotorConfig.BuddyBalanceRight1);
    rightMotor2 = MotorController.constructMotor(MotorConfig.BuddyBalanceRight2);
    leftMotor1 = MotorController.constructMotor(MotorConfig.BuddyBalanceLeft1);
    leftMotor2 = MotorController.constructMotor(MotorConfig.BuddyBalanceLeft2);
    rightMotor1.setSmartCurrentLimit(10);
    rightMotor2.setSmartCurrentLimit(10);
    leftMotor1.setSmartCurrentLimit(10);
    leftMotor2.setSmartCurrentLimit(10);
    deployServo1 = new Servo(deployServo1ID);
    deployServo2 = new Servo(deployServo2ID);
    
    encoder = AbsoluteEncoder.constructREVEncoder(EncoderConfig.BuddyBalance);
    unifiedPIDController = new ProfiledPIDController(kDefaultMotorP, kDefaultMotorI, kDefaultMotorD, kConstraints);

    tunerNumP = new TunableNumber("Buddy Balance Motor P", kDefaultMotorP, unifiedPIDController::setP);
    tunerNumI = new TunableNumber("Buddy Balance Motor I", kDefaultMotorI, unifiedPIDController::setI);
    tunerNumD = new TunableNumber("Buddy Balance Motor D", kDefaultMotorD, unifiedPIDController::setD);

    buddyBalancePosEntry.setString("Docked");
  }

  public double getDesiredAngle() {
    return Math.round(AbsoluteEncoder.getPositionRadians(encoder)*100)/100.0;
  }

  public boolean getIsDeployed() {
    return isDeployed;
  }

  public void deployBuddyBalance() { // Removes servo locking mechanism, releasing gas shocks
    deployServo1.set(kServoDeployedPos1);
    deployServo2.set(kServoDeployedPos2);
    rightMotor1.setSmartCurrentLimit(60);
    rightMotor2.setSmartCurrentLimit(60);
    leftMotor1.setSmartCurrentLimit(60);
    leftMotor2.setSmartCurrentLimit(60);
    isDeployed = true;
    buddyBalancePosEntry.setString("Deployed");
    servoPositionEntry.setDouble(deployServo1.get());
  }

  public void returnServos() { // Returns servos to the docked position
    deployServo1.set(kServoInitialPos);
    deployServo2.set(kServoInitialPos);
    buddyBalancePosEntry.setString("Docked");
    servoPositionEntry.setDouble(deployServo1.get());
  }

  public void retrieveBuddy() { // Used to pick up the buddy robot while the lift is already underneath it
    unifiedPIDController.setGoal(Units.degreesToRadians(kRetrievedAngle));
    buddyBalancePosEntry.setString("Raised");
  }

  public void releaseBuddy() { // Used to set down the robot
    unifiedPIDController.setGoal(Units.degreesToRadians(kDeployedAngle));
    buddyBalancePosEntry.setString("Lowered");
  }

  public void updateMotors() {
    rightMotor1.set(MathUtil.clamp(unifiedPIDController.calculate(getDesiredAngle()), -1, 1));
    rightMotor2.set(MathUtil.clamp(unifiedPIDController.calculate(getDesiredAngle()), -1, 1));
    leftMotor1.set(MathUtil.clamp(unifiedPIDController.calculate(getDesiredAngle()), -1, 1));
    leftMotor2.set(MathUtil.clamp(unifiedPIDController.calculate(getDesiredAngle()), -1, 1));
    rightMotorOutputEntry.setDouble(rightMotor1.get());
    leftMotorOutputEntry.setDouble(leftMotor1.get());
    setpointEntry.setDouble(unifiedPIDController.getGoal().position);
  }

  // These methods are used for JUnit testing only
  public void close() throws Exception {
    // This method will close all device handles used by this object and release any other dynamic memory.
    // Mostly for JUnit tests
    rightMotor1.close();
    rightMotor2.close();
    leftMotor1.close();
    leftMotor2.close();
    deployServo1.close();
    deployServo2.close();
    encoder.close();
  }

  public double getActualServoPosition1() {
    return deployServo1.get();
  }
  public double getActualServoPosition2() {
    return deployServo2.get();
  }

  public static double getDesiredServoPosition1() {
    return kServoDeployedPos1;
  }

  public static double getDesiredServoPosition2() {
    return kServoDeployedPos2;
  }

  public double getActualEncoderAngle() {
    return encoder.getAbsolutePosition();
  }

  public static double getCalculatedEncoderAngle() {
    return encoderCalculatedAngle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run 
    updateMotors();
    buddyBalancePos.append(AbsoluteEncoder.getPositionRadians(encoder)); // Logging the position of the buddy balance lift
    positionEntry.setDouble(AbsoluteEncoder.getPositionRadians(encoder));
    encoderCalculatedAngle = unifiedPIDController.calculate(getDesiredAngle());
    SmartDashboard.putNumber("Buddy Balance Angle", Units.radiansToDegrees(AbsoluteEncoder.getPositionRadians(encoder)));
    // to prevent the world from ending (cam breaking swerve modules)
    if ((AbsoluteEncoder.getPositionRadians(encoder) > Math.PI) || (AbsoluteEncoder.getPositionRadians(encoder) < 0)) {
      rightMotor1.set(0);
      rightMotor2.set(0);
      leftMotor1.set(0);
      leftMotor2.set(0);
      unifiedPIDController.setGoal(Units.degreesToRadians(90));
    }
  }
}