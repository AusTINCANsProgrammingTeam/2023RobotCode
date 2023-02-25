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
  private static ShuffleboardTab buddyBalanceTab = Shuffleboard.getTab("Buddy Balance"); // TODO: Replace buddy balance tab with whatever tab the position should be logged to
  private static GenericEntry positionEntry = buddyBalanceTab.add("Buddy Balance Position", 0).getEntry();
  private static ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
  private static GenericEntry buddyBalancePosEntry = matchTab.add("Buddy Balance State", "Docked").getEntry();
  // TODO: Uncomment comp shuffleboard objects when both comp-shuffleboard and buddy-balance-encoder pull requests go through

  private static double encoderActualRightAngle;
  private static double encoderActualLeftAngle;
  private static double encoderCalculatedRightAngle; 
  private static double encoderCalculatedLeftAngle;
  private static double servoActualPosition;

  public BuddyBalanceSubsystem() {
    rightMotor = MotorController.constructMotor(MotorConfig.BuddyBalanceRight);
    leftMotor = MotorController.constructMotor(MotorConfig.BuddyBalanceLeft);
    activateDeploy = new Servo(deployServoID);
    rightEncoder = AbsoluteEncoder.constructREVEncoder(EncoderConfig.BuddyBalanceRight);
    leftEncoder = AbsoluteEncoder.constructREVEncoder(EncoderConfig.BuddyBalanceLeft);
    rightPIDController = new PIDController(kDefaultMotorP, kDefaultMotorI, kDefaultMotorD);
    leftPIDController = new PIDController(kDefaultMotorP, kDefaultMotorI, kDefaultMotorD);

    tunerNumRightP = new TunableNumber("Buddy Balance Right Motor P", kDefaultMotorP, rightPIDController::setP);
    tunerNumRightI = new TunableNumber("Buddy Balance Right Motor I", kDefaultMotorI, rightPIDController::setI);
    tunerNumRightD = new TunableNumber("Buddy Balance Right Motor D", kDefaultMotorD, rightPIDController::setD);
    tunerNumLeftP = new TunableNumber("Buddy Balance Left Motor P", kDefaultMotorP, leftPIDController::setP);
    tunerNumLeftI = new TunableNumber("Buddy Balance Left Motor I", kDefaultMotorI, leftPIDController::setI);
    tunerNumLeftD = new TunableNumber("Buddy Balance Left Motor D", kDefaultMotorD, leftPIDController::setD);

    refPointBalancedTuner = new TunableNumber("Ref Point Balanced", kRetrievedAngle, (a) -> {tunedRetrievedAngle = Units.degreesToRadians(a);});
    refPointDeployedTuner = new TunableNumber("Ref Point Deployed", kDeployedAngle, (a) -> {tunedDeployedAngle = Units.degreesToRadians(a);});
    refPointServoTuner = new TunableNumber("Ref Point Servo", kServoDeployedPos, (a) -> {tunedServoDeployedPos = Units.degreesToRadians(a);});
  }

  public double getRightAngle() {
    return Math.round(AbsoluteEncoder.getPositionRadians(rightEncoder)*100)/100.0;
  }

  public double getLeftAngle() {
    return Math.round(AbsoluteEncoder.getPositionRadians(leftEncoder)*100)/100.0;
  }

  public boolean getIsDeployed() {
    return isDeployed;
  }

  public void deployBuddyBalance() {
    activateDeploy.set(kServoDeployedPos); // Change constants to the tuned values while testing (kServoDeployedPos becomes tunedServoDeployedPos)
    isDeployed = true;
  }

  public void retrieveBuddy() { // Used to pick up the buddy robot while the lift is already underneath it
    rightPIDController.setSetpoint(kRetrievedAngle); // Change constants to the tuned values while testing (KRetrievedAngle becomes tunedRetrievedAngle)
    leftPIDController.setSetpoint(kRetrievedAngle);
    //buddyBalancePosEntry.setString("Raised");
  }

  public void releaseBuddy() { // Used to set down the robot
    rightPIDController.setSetpoint(kDeployedAngle); // Change constants to the tuned values while testing (kDeployedAngle becomes tunedDeployedAngle)
    leftPIDController.setSetpoint(kDeployedAngle);
    //buddyBalancePosEntry.setString("Lowered");
  }

  public void updateMotors() {
    encoderCalculatedRightAngle = rightPIDController.calculate(getRightAngle());
    encoderCalculatedLeftAngle = leftPIDController.calculate(getLeftAngle());
    rightMotor.set(MathUtil.clamp(encoderCalculatedRightAngle, -1, 1));
    leftMotor.set(MathUtil.clamp(encoderCalculatedLeftAngle, -1, 1));
  }

  // These methods are used for JUnit testing only
  public void close() throws Exception {
    // This method will close all device handles used by this object and release any other dynamic memory.
    // Mostly for JUnit tests
    rightMotor.close();
    leftMotor.close();
    activateDeploy.close();
    rightEncoder.close();
    leftEncoder.close();
  }

  public double getActualServoPosition() {
    return servoActualPosition;
  }

  public static double getDesiredServoPosition() {
    return kServoDeployedPos;
  }

  public double getActualEncoderRightAngle() {
    return encoderActualRightAngle;
  }

  public double getActualEncoderLeftAngle() {
    return encoderActualLeftAngle;
  }

  public static double getCalculatedEncoderRightAngle() {
    return encoderCalculatedRightAngle;
  }

  public static double getCalculatedEncoderLeftAngle() {
    return encoderCalculatedLeftAngle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    buddyBalancePosLeft.append(AbsoluteEncoder.getPositionRadians(leftEncoder)); // Logging the position of the buddy balance lift
    buddyBalancePosRight.append(AbsoluteEncoder.getPositionRadians(rightEncoder));
    encoderActualRightAngle = rightEncoder.getAbsolutePosition();
    encoderActualLeftAngle = leftEncoder.getAbsolutePosition();
    servoActualPosition = activateDeploy.get();
    SmartDashboard.putNumber("Buddy Balance Right Angle", Units.radiansToDegrees(AbsoluteEncoder.getPositionRadians(rightEncoder)));
    SmartDashboard.putNumber("Buddy Balance Left Angle", Units.radiansToDegrees(AbsoluteEncoder.getPositionRadians(leftEncoder)));
  }
}