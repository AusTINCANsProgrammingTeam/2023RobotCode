// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
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
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.classes.TunableNumber;
import frc.robot.hardware.AbsoluteEncoder;
import frc.robot.hardware.AbsoluteEncoder.EncoderConfig;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;


public class ArmSubsystem extends SubsystemBase {
  public static enum ArmState{
    STOWED(0.5756, 0.0280), //Arm is retracted into the frame perimeter
    CONEINTAKE(1.0136, -0.0749), //Arm is in position to intake cones
    CUBEINTAKE(0.7984, -0.2416), //Arm is in position to intake cubes
    MIDSCORE(1.4536, 0.9486), //Arm is in position to score on the mid pole
    HIGHSCORE(1.6324, 1.3305), //Arm is in position to score on the high pole
    HIGHTRANSITION(1.2283,1.0732), //Used as an intermediate step when in transition to high score
    HIGHDROP(1.4433, 0.8766), //High scoring motion
    TRANSITION(0.7124, 0.1644); //Used to transition to any state from stowed position

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

  //Base arm PID values
  private double kBaseP = 1;
  private double kBaseI = 0.35;
  private double kBaseD = 0;
  //Elbow arm PID values
  private double kElbowP = 1.5;
  private double kElbowI = 0.25;
  private double kElbowD = 0;
  //Sim PID values
  private double kSimBaseP = 0.1;
  private double kSimElbowP = 0.1;

  private double armXPosition;
  private double armYPosition;

  private CANSparkMax baseMotor;
  private CANSparkMax baseMotor2;
  private CANSparkMax elbowMotor;

  private final DutyCycleEncoder baseAbsoluteEncoder;
  private final DutyCycleEncoder choochooAbsoluteEncoder;
  private final DutyCycleEncoder elbowAbsoluteEncoder;
  private double simBaseEncoderPosition;
  private double simElbowEncoderPosition;

  private final ProfiledPIDController basePIDController;
  private final ProfiledPIDController elbowPIDController;

  public static final double kMinChooChooAngle = Units.degreesToRadians(208);
  public static final double kMaxChooChooAngle = Units.degreesToRadians(326);

  public static final double kBaseLength = Units.inchesToMeters(41);
  public static final double kElbowLength = Units.inchesToMeters(43);

  public static final double kMinBaseAngle = Units.degreesToRadians(46);
  public static final double kMinElbowAngle = Units.degreesToRadians(15);
  public static final double kMaxBaseAngle = Units.degreesToRadians(90);
  public static final double kMaxElbowAngle = Units.degreesToRadians(162);

  public static final Constraints kBaseConstraints = new Constraints(Units.degreesToRadians(80), Units.degreesToRadians(80));
  public static final Constraints kElbowConstraints = new Constraints(Units.degreesToRadians(180), Units.degreesToRadians(180));

  public static final double kBaseGearing = 40.8333333;
  public static final double kElbowGearing = 4.28571429;
  public static final double kBaseMass = Units.lbsToKilograms(20);
  public static final double kElbowMass = Units.lbsToKilograms(5);
  public static final double kBaseInertia = SingleJointedArmSim.estimateMOI(kBaseLength, kBaseMass);
  public static final double kElbowInertia = SingleJointedArmSim.estimateMOI(kElbowLength, kElbowMass);

  //Real arm values
  private ShuffleboardTab armTab;
  private ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
  private ShuffleboardTab configTab = Shuffleboard.getTab("Config");

  private GenericEntry actualBaseAngle;
  private GenericEntry actualChooChooAngle;
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
    //Add coast mode command to shuffleboard
    configTab.add(new StartEndCommand(this::coastBase, this::brakeBase, this).ignoringDisable(true).withName("Coast Arm"));

    baseMotor = MotorController.constructMotor(MotorConfig.ArmBase1);
    baseMotor2 = MotorController.constructMotor(MotorConfig.ArmBase2);
    elbowMotor = MotorController.constructMotor(MotorConfig.ArmElbow);

    baseMotor2.follow(baseMotor);

    baseMotor.enableVoltageCompensation(11);
    elbowMotor.enableVoltageCompensation(11);

    baseAbsoluteEncoder = AbsoluteEncoder.constructREVEncoder(EncoderConfig.ArmBase);
    elbowAbsoluteEncoder = AbsoluteEncoder.constructREVEncoder(EncoderConfig.ArmElbow);
    choochooAbsoluteEncoder = AbsoluteEncoder.constructREVEncoder(EncoderConfig.ArmChooChoo);

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
      SmartDashboard.putData("Arm Sim", simArmCanvas);
      basePIDController = new ProfiledPIDController(kSimBaseP, 0, 0, kBaseConstraints);
      elbowPIDController = new ProfiledPIDController(kSimElbowP, 0, 0, kElbowConstraints);
    }

    if(!Robot.isCompetition){
      armTab = Shuffleboard.getTab("Arm");

      actualBaseAngle = armTab.add("Actual Base Angle", 0.0).getEntry();
      actualChooChooAngle = armTab.add("Actual Choo Choo Angle", 0.0).getEntry();
      desiredBaseGoal = armTab.add("Desired Base Goal", 0.0).getEntry();
      desiredBaseSetpoint = armTab.add("Desired Base Setpoint", 0.0).getEntry();
      actualElbowAngle = armTab.add("Actual Elbow Angle", 0.0).getEntry();
      desiredElbowGoal = armTab.add("Desired Elbow Goal", 0.0).getEntry();
      desiredElbowSetpoint = armTab.add("Desired Elbow Setpoint", 0.0).getEntry();

      actualXPosition = armTab.add("Actual X Position", 0).getEntry();
      actualYPositon = armTab.add("Actual Y Position", 0).getEntry();

      desiredXPosition = armTab.add("Desired X Position", 0.0).getEntry();
      desiredYPosition = armTab.add("Desired Y Position", 0.0).getEntry();
    }

    basePIDController.reset(getBaseAngle());
    elbowPIDController.reset(getElbowAngle());

    holdCurrentPosition();
    
    setState(kDefaultState);
  }

  //Returns sim encoder position (No offset) if in simulation, the actual position otherwise
  public double getBaseAngle() {
    return Robot.isSimulation() ? simBaseEncoderPosition : AbsoluteEncoder.getPositionRadians(baseAbsoluteEncoder,3);
  }

  public double getElbowAngle() {
    return Robot.isSimulation() ? simElbowEncoderPosition : AbsoluteEncoder.getPositionRadians(elbowAbsoluteEncoder,3);
  }

  public double getChooChooAngle() {
    return AbsoluteEncoder.getPositionRadians(choochooAbsoluteEncoder,3);
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
    double baseOutput = MathUtil.clamp(((getChooChooAngle() < kMaxChooChooAngle && getChooChooAngle() > kMinChooChooAngle) ? -1 : 1) * basePIDController.calculate(getBaseAngle()),-1,1);
    double elbowOutput = MathUtil.clamp(elbowPIDController.calculate(getElbowAngle()),0,1);
    baseMotor.set(baseOutput);
    elbowMotor.set(elbowOutput);
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
    double desiredElbowAngle = convertToElbowAngle(x, y);

    if(!Robot.isCompetition){
      desiredXPosition.setDouble(x);
      desiredYPosition.setDouble(y);
    }
    
    setBaseReference(desiredBaseAngle);
    setElbowReference(desiredElbowAngle);
  }

  public void setState(ArmState state){
    currentStateEntry.setString(state.toString());
    currentState = state;

    basePIDController.setTolerance(state == ArmState.TRANSITION ? 10 : 0.25);
    elbowPIDController.setTolerance(state == ArmState.TRANSITION ? 10 : 0.25);

    setDesiredPositions(state.getX(), state.getY());
  }

  public void holdCurrentPosition(){
    setBaseReference(getBaseAngle());
    setElbowReference(getElbowAngle());
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
    return new SequentialCommandGroup(
      goToState(ArmState.TRANSITION),
      goToState(state)
    ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command handleHighButton(){
    switch(currentState){
      case STOWED:
      case TRANSITION:
      case CONEINTAKE:
      case CUBEINTAKE:
      case MIDSCORE:
        return goToState(ArmState.HIGHTRANSITION);
      case HIGHSCORE:
        return goToState(ArmState.HIGHDROP);
      case HIGHTRANSITION:
        return goToState(ArmState.HIGHSCORE);
      case HIGHDROP:
        return transitionToState(ArmState.STOWED);
      default:
        return null;
    }
  }

  public Command handleMidButton(){
    switch(currentState){
      case STOWED:
      case TRANSITION:
      case CONEINTAKE:
      case CUBEINTAKE:
      case HIGHDROP:
      case HIGHTRANSITION:
        return goToState(ArmState.MIDSCORE);
      case MIDSCORE:
        return goToState(ArmState.STOWED);
      case HIGHSCORE:
        return goToState(ArmState.HIGHDROP);
      default:
        return null;
    }
  }

  public Command handleConeIntakeButton(){
    switch(currentState){
      case STOWED:
        return transitionToState(ArmState.CONEINTAKE);
      case CONEINTAKE:
        return transitionToState(ArmState.STOWED);
      case HIGHSCORE:
        return goToState(ArmState.HIGHDROP);
      case TRANSITION:
      case CUBEINTAKE:
      case MIDSCORE:
      case HIGHTRANSITION:
      case HIGHDROP:
        return goToState(ArmState.CONEINTAKE);
      default:
        return null;
    }
  }

  public Command handleCubeIntakeButton(){
    switch(currentState){
      case STOWED:
        return transitionToState(ArmState.CUBEINTAKE);
      case HIGHTRANSITION:
      case CUBEINTAKE:
        return transitionToState(ArmState.STOWED);
      case HIGHSCORE:
        return goToState(ArmState.HIGHDROP);
      case TRANSITION:
      case CONEINTAKE:
      case MIDSCORE:
      case HIGHDROP:
        return goToState(ArmState.CUBEINTAKE);
      default:
        return null;
    }
  }

  public Command highScoreSequence() {
    return new SequentialCommandGroup(
      goToState(ArmState.HIGHTRANSITION),
      goToState(ArmState.HIGHSCORE),
      goToState(ArmState.HIGHDROP)
    );
  }

  public Command stowArmParallel() {
    //Run along with a trajectory to stow arm after scoring
    return new WaitCommand(0.5).andThen(goToState(ArmState.STOWED));
  }

  public void coastBase() {
    baseMotor.setIdleMode(IdleMode.kCoast);
    baseMotor2.setIdleMode(IdleMode.kCoast);
  }

  public void brakeBase() {
    baseMotor.setIdleMode(IdleMode.kBrake);
    baseMotor2.setIdleMode(IdleMode.kBrake);
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
    SmartDashboard.putBoolean("Safety", getChooChooAngle() > kMinChooChooAngle && getChooChooAngle() < kMaxChooChooAngle);
    // This method will be called once per scheduler run
    calculateCurrentPositions();
    if(DriverStation.isDisabled()){
      holdCurrentPosition();
    }

    if(!Robot.isCompetition){
      //Shuffleboard + Smartdashboard values 
      actualBaseAngle.setDouble(Units.radiansToDegrees(getBaseAngle()));
      actualChooChooAngle.setDouble(Units.radiansToDegrees(getChooChooAngle()));
      actualElbowAngle.setDouble(Units.radiansToDegrees(getElbowAngle()));
      actualXPosition.setDouble(armXPosition);
      actualYPositon.setDouble(armYPosition);

      desiredBaseGoal.setDouble(Units.radiansToDegrees(basePIDController.getGoal().position));
      desiredBaseSetpoint.setDouble(Units.radiansToDegrees(basePIDController.getSetpoint().position));
      desiredElbowGoal.setDouble(Units.radiansToDegrees(elbowPIDController.getGoal().position));
      desiredElbowSetpoint.setDouble(Units.radiansToDegrees(elbowPIDController.getSetpoint().position));
    }
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