// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;
import frc.robot.Robot;
import frc.robot.classes.TunableNumber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class BuddyBalanceSubsystem extends SubsystemBase {
  // Constants
  private static double kBalancedPosition; // Buddy balance PID reference point when lifting a robot and engaging charge station
  private static double kDeployedPosition; // Buddy balance PID reference point when setting down a robot/initial position when deployed
  // TODO: Make the reference point constants and default motor PID values final when they are done being tuned with TunableNumbers
  private static final double kDefaultMotorP = 1e-6;
  private static final double kDefaultMotorI = 0;
  private static final double kDefaultMotorD = 1e-6;
  private static final int deployServoID = 1;
  private static double kServoDeployedPos = 1;

  // Motors
  private Servo activateDeploy;
  private CANSparkMax rightMotor;
  private CANSparkMax leftMotor;
  private SparkMaxPIDController rightPIDController;
  private SparkMaxPIDController leftPIDController;
  private RelativeEncoder rightEncoder;
  private RelativeEncoder leftEncoder;

  // Simulation
  private double kSimP = 0.1;
  private double kSimI = 0.0;
  private double kSimD = 0.0;
  private double kSimGearing = 0;
  private double kArmLength = Units.inchesToMeters(43.5);
  private double kArmMass = 2;
  private double kMinAngle = 70;
  private double kMaxAngle = 90;
  private double simBBEncoderPosition;
  private final double kSimInertia = SingleJointedArmSim.estimateMOI(kArmLength, kArmMass);
  private final SingleJointedArmSim buddyBalanceSim = new SingleJointedArmSim(DCMotor.getNEO(1), kSimGearing, kSimInertia, kArmLength, kMinAngle, kMaxAngle, false);
  private Mechanism2d simBBCanvas = new Mechanism2d(5, 5);
  private MechanismRoot2d buddyBalanceSimRoot = simBBCanvas.getRoot("Buddy Balance Arm Root", 0, 0);
  private MechanismLigament2d buddyBalanceSimV = buddyBalanceSimRoot.append(new MechanismLigament2d("Buddy Balance Lift Arm", kArmLength*3, buddyBalanceSim.getAngleRads()));

  // Tunable Numbers
  private TunableNumber refPointBalancedTuner;
  private TunableNumber refPointDeployedTuner;
  private TunableNumber refPointServoTuner;
  private TunableNumber tunerNumRightP;
  private TunableNumber tunerNumRightI;
  private TunableNumber tunerNumRightD;
  private TunableNumber tunerNumLeftP;
  private TunableNumber tunerNumLeftI;
  private TunableNumber tunerNumLeftD;

  // Shuffleboard
  private boolean isDeployed = false;
  private DataLog datalog = DataLogManager.getLog();
  private DoubleLogEntry buddyBalancePosition = new DoubleLogEntry(datalog, "/buddybalance/position");
  private ShuffleboardTab buddyBalanceTab = Shuffleboard.getTab("Buddy Balance"); // TODO: Replace buddy balance tab with whatever tab the position should be logged to
  private GenericEntry positionEntry;
  private ShuffleboardTab buddyBalanceSimTab = Shuffleboard.getTab("Buddy Balance Simulation");
  private GenericEntry simBBAngle = buddyBalanceSimTab.add("Sim BB Arm Angle", 0.0).getEntry();
  private GenericEntry simBBEncoderPos = buddyBalanceSimTab.add("Sim BB Encoder Angle", 0.0).getEntry();
  private GenericEntry simBBOutput = buddyBalanceSimTab.add("Sim BB Output", 0.0).getEntry();

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

    tunerNumRightP = new TunableNumber("Buddy Balance Right Motor P", kDefaultMotorP, rightPIDController::setP);
    tunerNumRightI = new TunableNumber("Buddy Balance Right Motor I", kDefaultMotorI, rightPIDController::setI);
    tunerNumRightD = new TunableNumber("Buddy Balance Right Motor D", kDefaultMotorD, rightPIDController::setD);
    tunerNumLeftP = new TunableNumber("Buddy Balance Left Motor P", kDefaultMotorP, leftPIDController::setP);
    tunerNumLeftI = new TunableNumber("Buddy Balance Left Motor I", kDefaultMotorI, leftPIDController::setI);
    tunerNumLeftD = new TunableNumber("Buddy Balance Left Motor D", kDefaultMotorD, leftPIDController::setD);

    refPointBalancedTuner = new TunableNumber("Ref Point Balanced", 15, (a) -> {kBalancedPosition = a;});
    refPointDeployedTuner = new TunableNumber("Ref Point Deployed", 0, (a) -> {kDeployedPosition = a;});
    refPointServoTuner = new TunableNumber("Ref Point Servo", 1, (a) -> {kServoDeployedPos = a;});

    positionEntry = buddyBalanceTab.add("Buddy Balance Position", 0).getEntry();
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
  public void simulationPeriodic() {
    simBBEncoderPosition = buddyBalanceSim.getAngleRads();
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(buddyBalanceSim.getCurrentDrawAmps()));
    buddyBalanceSimV.setAngle(Units.radiansToDegrees(buddyBalanceSim.getAngleRads()));
    buddyBalanceSim.update(Robot.kDefaultPeriod);

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    buddyBalancePosition.append(rightEncoder.getPosition()); // Logging the position of the buddy balance lift
    buddyBalancePosition.append(leftEncoder.getPosition());
  }
}