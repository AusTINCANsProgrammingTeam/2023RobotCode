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
  private static final double kRetrievedAngle = 0; // Buddy balance PID reference point when lifting a robot and engaging charge station
  private static final double kDeployedAngle = 0; // Buddy balance PID reference point when setting down a robot/initial position when deployed
  // TODO: Make the constants final when they are done being tuned with TunableNumbers
  private static final double kDefaultMotorP = 1e-6;
  private static final double kDefaultMotorI = 0;
  private static final double kDefaultMotorD = 1e-6;
  private static final int deployServoID = 1;
  private static final double kServoDeployedPos = 0;
  private double tunedRetrievedAngle;
  private double tunedDeployedAngle;
  private double tunedServoDeployedPos;

  private TunableNumber refPointBalancedTuner;
  private TunableNumber refPointDeployedTuner;
  private TunableNumber refPointServoTuner;
  private TunableNumber tunerNumP;
  private TunableNumber tunerNumI;
  private TunableNumber tunerNumD;

  private Servo deployServo;
  private CANSparkMax rightMotor;
  private CANSparkMax leftMotor;
  private PIDController unifiedPIDController; // Never forget
  private DutyCycleEncoder encoder;
  private boolean isDeployed = false;
  private DataLog datalog = DataLogManager.getLog();
  private DoubleLogEntry buddyBalancePos = new DoubleLogEntry(datalog, "/buddybalance/position/");
  private static ShuffleboardTab buddyBalanceTab = Shuffleboard.getTab("Buddy Balance"); // TODO: Replace buddy balance tab with whatever tab the position should be logged to
  private static GenericEntry positionEntry = buddyBalanceTab.add("Buddy Balance Position", 0).getEntry();
  private static ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
  private static GenericEntry buddyBalancePosEntry = matchTab.add("Buddy Balance State", "Docked").getEntry();
  // TODO: Uncomment comp shuffleboard objects when both comp-shuffleboard and buddy-balance-encoder pull requests go through

  private static double encoderCalculatedAngle; 

  public BuddyBalanceSubsystem() {
    rightMotor = MotorController.constructMotor(MotorConfig.BuddyBalanceRight);
    leftMotor = MotorController.constructMotor(MotorConfig.BuddyBalanceLeft);
    deployServo = new Servo(deployServoID);
    encoder = AbsoluteEncoder.constructREVEncoder(EncoderConfig.BuddyBalance);
    unifiedPIDController = new PIDController(kDefaultMotorP, kDefaultMotorI, kDefaultMotorD);

    tunerNumP = new TunableNumber("Buddy Balance Motor P", kDefaultMotorP, unifiedPIDController::setP);
    tunerNumI = new TunableNumber("Buddy Balance Motor I", kDefaultMotorI, unifiedPIDController::setI);
    tunerNumD = new TunableNumber("Buddy Balance Motor D", kDefaultMotorD, unifiedPIDController::setD);

    refPointBalancedTuner = new TunableNumber("Ref Point Balanced", kRetrievedAngle, (a) -> {tunedRetrievedAngle = Units.degreesToRadians(a);});
    refPointDeployedTuner = new TunableNumber("Ref Point Deployed", kDeployedAngle, (a) -> {tunedDeployedAngle = Units.degreesToRadians(a);});
    refPointServoTuner = new TunableNumber("Ref Point Servo", kServoDeployedPos, (a) -> {tunedServoDeployedPos = Units.degreesToRadians(a);});
  }

  public double getDesiredAngle() {
    return Math.round(AbsoluteEncoder.getPositionRadians(encoder)*100)/100.0;
  }

  public boolean getIsDeployed() {
    return isDeployed;
  }

  public void deployBuddyBalance() {
    deployServo.set(tunedServoDeployedPos); // TODO: Change tuned value to constant when done testing (tunedServoDeployedPos becomes kServoDeployedPos)
    isDeployed = true;
  }

  public void retrieveBuddy() { // Used to pick up the buddy robot while the lift is already underneath it
    unifiedPIDController.setSetpoint(tunedRetrievedAngle); // TODO: Change tuned value to constant when done testing (tunedRetrievedAngle becomes kRetrievedAngle)
    buddyBalancePosEntry.setString("Raised");
  }

  public void releaseBuddy() { // Used to set down the robot
    unifiedPIDController.setSetpoint(tunedDeployedAngle); // TODO: Change tuned value to constant when done testing (tunedDeployedAngle becomes kDeployedAngle)
    buddyBalancePosEntry.setString("Lowered");
  }

  public void updateMotors() {
    rightMotor.set(MathUtil.clamp(unifiedPIDController.calculate(getDesiredAngle()), -1, 1));
    leftMotor.set(MathUtil.clamp(unifiedPIDController.calculate(getDesiredAngle()), -1, 1));
  }

  // These methods are used for JUnit testing only
  public void close() throws Exception {
    // This method will close all device handles used by this object and release any other dynamic memory.
    // Mostly for JUnit tests
    rightMotor.close();
    leftMotor.close();
    deployServo.close();
    encoder.close();
  }

  public double getActualServoPosition() {
    return deployServo.get();
  }

  public static double getDesiredServoPosition() {
    return kServoDeployedPos;
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
    buddyBalancePos.append(AbsoluteEncoder.getPositionRadians(encoder)); // Logging the position of the buddy balance lift
    encoderCalculatedAngle = unifiedPIDController.calculate(getDesiredAngle());
    SmartDashboard.putNumber("Buddy Balance Angle", Units.radiansToDegrees(AbsoluteEncoder.getPositionRadians(encoder)));
  }
}