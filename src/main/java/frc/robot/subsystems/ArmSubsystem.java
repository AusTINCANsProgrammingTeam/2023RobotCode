// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.classes.TunableNumber;
import frc.robot.hardware.AbsoluteEncoder;
import frc.robot.hardware.AbsoluteEncoder.EncoderConfig;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;


public class ArmSubsystem extends SubsystemBase {
  public static enum ArmState{
    STOWED(0.4406, 0.0397), //Arm is retracted into the frame perimeter FIXME
    CONEINTAKE(0.8099, -0.1106), //Arm is in position to intake cones FIXME
    CUBEINTAKE(0, 0), //Arm is in position to intake cubes FIXME
    SUBSTATIONINTAKE(1.6145, 1.0866), //Arm is in position to intake from substation FIXME
    MIDSCORE(1.3116, 0.7540), //Arm is in position to score on the mid pole FIXME
    HIGHSCORE(1.6685, 1.0699), //Arm is in position to score on the high pole FIXME
    HIGHTRANSITION(0,0),
    TRANSITION(0.7124, 0.1644); //Used to transition to any state from stowed position FIXME

    private double x; //Position relative to the base of the arm, in meters
    private double y; //Position above the carpet, in meters
    
    ArmState(double x, double y){
      this.x = x;
      this.y = y;
    }

    public double getX(){
      return x;
    }

    public double getY(){
      return y;
    }
  }

  private final ArmState kDefaultState = ArmState.STOWED;
  private ArmState currentState;
  private ArmState currentTransition;

  //Base arm PID values
  private double kBaseP = 0.8;
  private double kBaseI = 0.1;
  private double kBaseD = 0;
  //Elbow arm PID values
  private double kElbowP = 1.5;
  private double kElbowI = 0.1;
  private double kElbowD = 0.2;
  //Sim PID values
  private double kSimBaseP = 0.1;
  private double kSimElbowP = 0.1;

  private double armXPosition;
  private double armYPosition;

  private CANSparkMax baseMotor;
  private CANSparkMax baseMotor2;
  private CANSparkMax elbowMotor;

  private final DutyCycleEncoder baseAbsoluteEncoder;
  private final DutyCycleEncoder elbowAbsoluteEncoder;
  private double simBaseEncoderPosition;
  private double simElbowEncoderPosition;

  private final ProfiledPIDController basePIDController;
  private final ProfiledPIDController elbowPIDController;

  public static final double kBaseLength = Units.inchesToMeters(41);
  public static final double kElbowLength = Units.inchesToMeters(43);

  public static final double kMinBaseAngle = Units.degreesToRadians(46);
  public static final double kMinElbowAngle = Units.degreesToRadians(15);
  public static final double kMaxBaseAngle = Units.degreesToRadians(90);
  public static final double kMaxElbowAngle = Units.degreesToRadians(160);

  public static final Constraints kBaseConstraints = new Constraints(Units.degreesToRadians(30), Units.degreesToRadians(30));
  public static final Constraints kElbowConstraints = new Constraints(Units.degreesToRadians(60), Units.degreesToRadians(45));

  public static final double kBaseGearing = 40.8333333;
  public static final double kElbowGearing = 4.28571429;
  public static final double kBaseMass = Units.lbsToKilograms(20);
  public static final double kElbowMass = Units.lbsToKilograms(5);
  public static final double kBaseInertia = SingleJointedArmSim.estimateMOI(kBaseLength, kBaseMass);
  public static final double kElbowInertia = SingleJointedArmSim.estimateMOI(kElbowLength, kElbowMass);

  //Real arm values
  private ShuffleboardTab armTab;
  private ShuffleboardTab matchTab = Shuffleboard.getTab("Match");

  private GenericEntry actualBaseAngle;
  private GenericEntry desiredBaseGoal;
  private GenericEntry desiredBaseSetpoint;
  private GenericEntry actualElbowAngle;
  private GenericEntry desiredElbowGoal;
  private GenericEntry desiredElbowSetpoint;

  private GenericEntry actualXPosition;
  private GenericEntry actualYPositon;

  private GenericEntry desiredXPosition;
  private GenericEntry desiredYPosition;

  private GenericEntry currentStateEntry = matchTab.add("Current State","").getEntry();
  private GenericEntry currentTransitionEntry = matchTab.add("Current Transition","").getEntry();

  private TunableNumber basePTuner;
  private TunableNumber baseITuner;
  private TunableNumber baseDTuner;

  private TunableNumber elbowPTuner;
  private TunableNumber elbowITuner;
  private TunableNumber elbowDTuner;

  private final SingleJointedArmSim baseArmSim = new SingleJointedArmSim(DCMotor.getNEO(2), kBaseGearing, kBaseInertia, kBaseLength, kMinBaseAngle, kMaxBaseAngle, false);
  private final SingleJointedArmSim elbowArmSim = new SingleJointedArmSim(DCMotor.getNEO(1), kElbowGearing, kElbowInertia, kElbowLength, kMinElbowAngle, kMaxElbowAngle, false);

  //The "canvas" that both arms are drawn on
  Mechanism2d simArmCanvas = new Mechanism2d(7,7);
  //Where the base arm begins
  MechanismRoot2d baseRoot = simArmCanvas.getRoot("Base Arm Root", 0, 0);
  //Sets the base arm to the root, then the elbow arm to the end of the base arm
  MechanismLigament2d baseLigament = baseRoot.append(new MechanismLigament2d("Base Arm", kBaseLength*3, baseArmSim.getAngleRads()));
  MechanismLigament2d elbowLigament = baseLigament.append(new MechanismLigament2d("Elbow Arm", kElbowLength*3, elbowArmSim.getAngleRads()));
  

  public ArmSubsystem() {
    SmartDashboard.putData("Arm Sim", simArmCanvas);

    baseMotor = MotorController.constructMotor(MotorConfig.ArmBase1);
    baseMotor2 = MotorController.constructMotor(MotorConfig.ArmBase2);
    elbowMotor = MotorController.constructMotor(MotorConfig.ArmElbow);

    baseMotor2.follow(baseMotor);

    baseMotor.enableVoltageCompensation(11);
    elbowMotor.enableVoltageCompensation(11);

    baseAbsoluteEncoder = AbsoluteEncoder.constructREVEncoder(EncoderConfig.ArmBase);
    elbowAbsoluteEncoder = AbsoluteEncoder.constructREVEncoder(EncoderConfig.ArmElbow);

    if(Robot.isReal()) {
      basePIDController = new ProfiledPIDController(kBaseP, kBaseI, kBaseD, kBaseConstraints);
      elbowPIDController = new ProfiledPIDController(kElbowP, kElbowI, kElbowD, kElbowConstraints);
      
      basePTuner = new TunableNumber("baseP", kBaseP, basePIDController::setP);
      baseITuner = new TunableNumber("baseI", kBaseI, basePIDController::setI);
      baseDTuner = new TunableNumber("baseD", kBaseD, basePIDController::setD);
  
      elbowPTuner = new TunableNumber("elbowP", kElbowP, elbowPIDController::setP);
      elbowITuner = new TunableNumber("elbowI", kElbowI, elbowPIDController::setI);
      elbowDTuner = new TunableNumber("elbowD", kElbowD, elbowPIDController::setD);
    } else {
      basePIDController = new ProfiledPIDController(kSimBaseP, 0, 0, kBaseConstraints);
      elbowPIDController = new ProfiledPIDController(kSimElbowP, 0, 0, kElbowConstraints);
    }

    if(!Robot.isCompetition){
      armTab = Shuffleboard.getTab("Arm");

      actualBaseAngle = armTab.add("Actual Base Angle", 0.0).getEntry();
      desiredBaseGoal = armTab.add("Desired Base Goal", 0.0).getEntry();
      desiredBaseSetpoint = armTab.add("Desired Base Setpoint", 0.0).getEntry();
      actualElbowAngle = armTab.add("Actual Elbow Angle", 0.0).getEntry();
      desiredElbowGoal = armTab.add("Desired Elbow Goal", 0.0).getEntry();
      desiredElbowSetpoint = armTab.add("Desired Elbow Setpoint", 0.0).getEntry();

      actualXPosition = armTab.add("Actual X Position", 0).getEntry();
      actualYPositon = armTab.add("Actual Y Position", 0).getEntry();

      desiredXPosition = armTab.add("Desired X Position", 0.0).getEntry();
      desiredYPosition = armTab.add("Desired Y Position", 0.0).getEntry();

      currentStateEntry = armTab.add("Current State","").getEntry();
      currentTransitionEntry = armTab.add("Current Transition","").getEntry();
    }

    basePIDController.reset(getBaseAngle());
    elbowPIDController.reset(getElbowAngle());

    setBaseReference(getBaseAngle());
    setElbowReference(getElbowAngle());
    
    setState(kDefaultState);
    currentTransition = kDefaultState;
  }

  //Returns sim encoder position (No offset) if in simulation, the actual position otherwise
  public double getBaseAngle() {
    return Robot.isSimulation() ? simBaseEncoderPosition : Math.round(AbsoluteEncoder.getPositionRadians(baseAbsoluteEncoder)*1000)/1000.0;
  }

  public double getElbowAngle() {
    return Robot.isSimulation() ? simElbowEncoderPosition : Math.round(AbsoluteEncoder.getPositionRadians(elbowAbsoluteEncoder)*1000)/1000.0;
  }

  public void updateReferences(double bJoystickValue, double eJoystickValue) {
    setBaseReference(MathUtil.clamp(basePIDController.getGoal().position+Units.degreesToRadians(bJoystickValue),ArmSubsystem.kMinBaseAngle,ArmSubsystem.kMaxBaseAngle));
    setElbowReference(MathUtil.clamp(elbowPIDController.getGoal().position+Units.degreesToRadians(eJoystickValue),ArmSubsystem.kMinElbowAngle,ArmSubsystem.kMaxElbowAngle));
  }

  public void setBaseReference(double setpoint) {
    basePIDController.setGoal(setpoint);
  }

  public void setElbowReference(double setpoint) {
    elbowPIDController.setGoal(setpoint);
  }

  //Gets arm X and Y positions. Desmos simulation link: https://www.desmos.com/calculator/fv7smerzhp
  public double getRMagnitude() {
    //Use law of cosines to find the magnitude of the arm's resultant vector, in meters
    return Math.sqrt(Math.pow(kBaseLength, 2) + Math.pow(kElbowLength, 2) - 2 * kBaseLength * kElbowLength * Math.cos(getElbowAngle()));
  }

  public double getRBearing() {
    //Use law of sines to find the bearing of the arm's resultant vector, in radians
    return Math.asin((kElbowLength * Math.sin(getElbowAngle())) / getRMagnitude()) + (Math.PI - getBaseAngle());
  }

  public double getArmX() {
    return -1 * (getRMagnitude() * Math.cos(getRBearing()));
  }

  public double getArmY() {
    return getRMagnitude() * Math.sin(getRBearing());
  }

  //This updates the arm positions to be used for stuff like shuffleboard
  public void calculateCurrentPositions() {
    armXPosition = getArmX();
    armYPosition = getArmY();
  }
  

  public void updateMotors() {
    baseMotor.set(MathUtil.clamp(basePIDController.calculate(getBaseAngle()),-1,1));
    elbowMotor.set(MathUtil.clamp(elbowPIDController.calculate(getElbowAngle()),0,1));
  }


  public void updateSimMotors() {
    updateMotors();
    baseArmSim.setInputVoltage(baseMotor.get() * RobotController.getBatteryVoltage());
    elbowArmSim.setInputVoltage(elbowMotor.get() * RobotController.getBatteryVoltage());
  }

  //Angle finding methods
  //The documentation for these can be found at our notion page at
  //https://www.notion.so/2158/Arm-System-0fc38426812e42f2944f34668059c2e4
  public static double convertToPrelimAngle(double x, double y) {
    return (
      -1*Math.acos(
        ( (x*x) + (y*y) - (kBaseLength*kBaseLength) - (kElbowLength*kElbowLength) )/(2*kBaseLength*kElbowLength)
      )
    );
  }

  public static double convertToBaseAngle(double x, double y) {
    double pAngle = convertToPrelimAngle(x, y);
    return (
      Math.atan(y/x)-Math.atan((kElbowLength*Math.sin(pAngle) )/( (kBaseLength+(kElbowLength*Math.cos(pAngle))))
      )
    );
  }

  public static double convertToElbowAngle(double x, double y) {
    return Units.degreesToRadians(180) + convertToPrelimAngle(x, y);
  }

  public boolean atSetpoint(){
    return basePIDController.atGoal() && elbowPIDController.atGoal();
  }

  public void setDesiredPositions(double x, double y) {
    double desiredBaseAngle = convertToBaseAngle(x, y);
    double desiredElbowAngle = convertToBaseAngle(x, y);
    
    desiredXPosition.setDouble(x);
    desiredYPosition.setDouble(y);

    setBaseReference(desiredBaseAngle);
    setElbowReference(desiredElbowAngle);
  }

  public void setState(ArmState state){
    currentStateEntry.setString(state.toString());
    currentState = state;

    basePIDController.setTolerance(state == ArmState.TRANSITION ? 10 : 1);
    elbowPIDController.setTolerance(state == ArmState.TRANSITION ? 10 : 1);

    setDesiredPositions(state.getX(), state.getY());
  }
  
  public Command goToState(ArmState state){
    //Command for autonomous, obstructs routine until arm is at setpoint
    return new FunctionalCommand(
                () -> setState(state), //Init
                this::updateMotors, //Execute
                (b)->{}, //End 
                this::atSetpoint, //isFinished
                this
            );
  }

  public Command transitionToState(ArmState state){
    currentTransition = state;
    currentTransitionEntry.setString(currentTransition.toString());
    return new SequentialCommandGroup(
      goToState(ArmState.TRANSITION),
      goToState(state)
    ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command transitionToStateTeleop(ArmState state){
    if(state == currentTransition){
      return transitionToState(ArmState.STOWED);
    }
    if(state == ArmState.HIGHSCORE){
      if(currentTransition == ArmState.HIGHTRANSITION){
        currentTransition = state;
        return goToState(ArmState.HIGHSCORE);
      }
      if(currentTransition != ArmState.HIGHSCORE){
        return transitionToState(ArmState.HIGHTRANSITION);
      }
    }
    return transitionToState(state);
  }
  
  public void stop() {
    baseMotor.stopMotor();
    elbowMotor.stopMotor();
  }

  public void stopsim() {
    stop();
    baseArmSim.setInputVoltage(0);
    elbowArmSim.setInputVoltage(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    calculateCurrentPositions();

    //Shuffleboard + Smartdashboard values 
    actualBaseAngle.setDouble(Units.radiansToDegrees(getBaseAngle()));
    actualElbowAngle.setDouble(Units.radiansToDegrees(getElbowAngle()));
    actualXPosition.setDouble(armXPosition);
    actualYPositon.setDouble(armYPosition);

    desiredBaseGoal.setDouble(Units.radiansToDegrees(basePIDController.getGoal().position));
    desiredBaseSetpoint.setDouble(Units.radiansToDegrees(basePIDController.getSetpoint().position));
    desiredElbowGoal.setDouble(Units.radiansToDegrees(elbowPIDController.getGoal().position));
    desiredElbowSetpoint.setDouble(Units.radiansToDegrees(elbowPIDController.getSetpoint().position));
  }

  @Override
  public void simulationPeriodic() {
    updateSimMotors();

    simBaseEncoderPosition = baseArmSim.getAngleRads();
    simElbowEncoderPosition = elbowArmSim.getAngleRads();

    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(baseArmSim.getCurrentDrawAmps()+elbowArmSim.getCurrentDrawAmps()));

    baseLigament.setAngle(Units.radiansToDegrees(baseArmSim.getAngleRads()));
    elbowLigament.setAngle(Units.radiansToDegrees(elbowArmSim.getAngleRads())-baseLigament.getAngle()-90);
    
    baseArmSim.update(Robot.kDefaultPeriod); // standard loop time of 20ms
    elbowArmSim.update(Robot.kDefaultPeriod);
  }
}