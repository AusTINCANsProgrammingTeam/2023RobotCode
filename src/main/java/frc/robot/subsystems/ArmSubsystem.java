// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
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
import frc.robot.classes.DebugLog;
import frc.robot.classes.TunableNumber;
import frc.robot.hardware.AbsoluteEncoder;
import frc.robot.hardware.AbsoluteEncoder.EncoderConfig;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;


public class ArmSubsystem extends SubsystemBase {
  public static enum ArmState{
    STOWED(0.5756, 0.0280), //Arm is retracted into the frame perimeter
    CONEINTAKE(1.0136, -0.0876), //Arm is in position to intake cones
    CUBEINTAKE(0.7691, -0.2365), //Arm is in position to intake cubes
    MIDSCORE(1.4536, 0.9486), //Arm is in position to score on the mid pole
    HIGHSCORE(1.6773, 1.2778), //Arm is in position to score on the high pole
    HIGHTRANSITION(1.2283,1.0732), //Used as an intermediate step when in transition to high score
    HIGHDROP(1.4433, 0.9266), //High scoring motion
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
  private double kElbowUpP = 5;
  private double kElbowUpI = 0.5;
  private double kElbowUpD = 0;

  private double kElbowDownP = 5;
  private double kElbowDownI = 0.5;
  private double kElbowDownD = 0;

  private double kElbowS = 0;
  private double kElbowG = 1.43;
  private double kElbowV = 1.40;
  private double kElbowA = 0.11;
  //Sim PID values
  private double kSimBaseP = 0.1;
  private double kSimElbowP = 0.1;

  private double armXPosition;
  private double armYPosition;

  private CANSparkMax baseMotor;
  private CANSparkMax baseMotor2;
  private CANSparkMax elbowMotor;
  private CANSparkMax elbowMotor2;

  private final DutyCycleEncoder baseAbsoluteEncoder;
  private final DutyCycleEncoder choochooAbsoluteEncoder;
  private final DutyCycleEncoder elbowAbsoluteEncoder;
  private double simBaseEncoderPosition;
  private double simElbowEncoderPosition;

  private final ProfiledPIDController basePIDController;
  private final ProfiledPIDController elbowPIDController;
  private final ArmFeedforward elbowFeedForward;

  public static final double kMinChooChooAngle = Units.degreesToRadians(208);
  public static final double kMaxChooChooAngle = Units.degreesToRadians(326);

  public static final double kBaseLength = Units.inchesToMeters(41);
  public static final double kElbowLength = Units.inchesToMeters(43);

  public static final double kMinBaseAngle = Units.degreesToRadians(46);
  public static final double kMinElbowAngle = Units.degreesToRadians(22);
  public static final double kMaxBaseAngle = Units.degreesToRadians(90);
  public static final double kMaxElbowAngle = Units.degreesToRadians(170);

  public static final double kMaxElbowVoltage = 12;

  public static final Constraints kBaseConstraints = new Constraints(Units.degreesToRadians(133), Units.degreesToRadians(133));
  public static final Constraints kElbowConstraints = new Constraints(Units.degreesToRadians(300), Units.degreesToRadians(300));

  public static final double kBaseGearing = 40.8333333;
  public static final double kElbowGearing = 4.28571429;
  public static final double kBaseMass = Units.lbsToKilograms(20);
  public static final double kElbowMass = Units.lbsToKilograms(5);
  public static final double kBaseInertia = SingleJointedArmSim.estimateMOI(kBaseLength, kBaseMass);
  public static final double kElbowInertia = SingleJointedArmSim.estimateMOI(kElbowLength, kElbowMass);

  //Real arm values
  private ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
  private ShuffleboardTab configTab = Shuffleboard.getTab("Config");

  private DebugLog<Double> actualBaseAngleLog = new DebugLog<Double>(0.0, "Actual Base Angle", this);
  private DebugLog<Double> actualChooChooAngleLog = new DebugLog<Double>(0.0, "Actual Choo Choo Angle", this);
  private DebugLog<Double> desiredBaseGoalLog = new DebugLog<Double>(0.0, "Desired Base Goal", this);
  private DebugLog<Double> desiredBaseSetpointLog = new DebugLog<Double>(0.0, "Desired Base Setpoint", this);
  private DebugLog<Double> baseOutputLog = new DebugLog<Double>(0.0, "Base Output", this);

  private DebugLog<Double> actualElbowAngleLog = new DebugLog<Double>(0.0, "Actual Elbow Angle", this);
  private DebugLog<Double> desiredElbowGoalLog = new DebugLog<Double>(0.0, "Desired Elbow Goal", this);
  private DebugLog<Double> desiredElbowSetpointLog = new DebugLog<Double>(0.0, "Desired Elbow Setpoint", this);
  private DebugLog<Double> elbowOutputLog = new DebugLog<Double>(0.0, "Elbow Output", this);
  private DebugLog<Double> elbowPIDOutputLog = new DebugLog<Double>(0.0, "Elbow PID Output", this);
  private DebugLog<Double> elbowFFOutputLog = new DebugLog<Double>(0.0, "Elbow FF Output", this);
  private DebugLog<String> elbowUpDownLog = new DebugLog<String>("", "Elbow Up-Down", this);

  private DebugLog<Double> actualXPositionLog = new DebugLog<Double>(0.0, "Actual X Position", this);
  private DebugLog<Double> actualYPositionLog = new DebugLog<Double>(0.0, "Actual Y Position", this);

  private DebugLog<Double> desiredXPositionLog = new DebugLog<Double>(0.0, "Desired X Position", this);
  private DebugLog<Double> desiredYPositionLog = new DebugLog<Double>(0.0, "Desired Y Position", this);

  private DebugLog<Boolean> rolloverLog = new DebugLog<Boolean>(false, "Choo Choo Rollover", this);

  private GenericEntry currentStateEntry = matchTab.add("Current State","").getEntry();

  private TunableNumber basePTuner;
  private TunableNumber baseITuner;
  private TunableNumber baseDTuner;

  private TunableNumber elbowUpPTuner;
  private TunableNumber elbowUpITuner;
  private TunableNumber elbowUpDTuner;

  private TunableNumber elbowDownPTuner;
  private TunableNumber elbowDownITuner;
  private TunableNumber elbowDownDTuner;

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
    elbowMotor = MotorController.constructMotor(MotorConfig.ArmElbow1);
    elbowMotor2 = MotorController.constructMotor(MotorConfig.ArmElbow2);

    baseMotor2.follow(baseMotor);
    elbowMotor2.follow(elbowMotor);

    baseMotor.enableVoltageCompensation(11);
    elbowMotor.enableVoltageCompensation(11);

    baseAbsoluteEncoder = AbsoluteEncoder.constructREVEncoder(EncoderConfig.ArmBase);
    elbowAbsoluteEncoder = AbsoluteEncoder.constructREVEncoder(EncoderConfig.ArmElbow);
    choochooAbsoluteEncoder = AbsoluteEncoder.constructREVEncoder(EncoderConfig.ArmChooChoo);

    if(Robot.isReal()) {
      basePIDController = new ProfiledPIDController(kBaseP, kBaseI, kBaseD, kBaseConstraints);
      elbowPIDController = new ProfiledPIDController(kElbowUpP, kElbowUpI, kElbowUpD, kElbowConstraints);
      elbowFeedForward = new ArmFeedforward(kElbowS, kElbowG, kElbowV, kElbowA);
      
      basePTuner = new TunableNumber("baseP", kBaseP, basePIDController::setP);
      baseITuner = new TunableNumber("baseI", kBaseI, basePIDController::setI);
      baseDTuner = new TunableNumber("baseD", kBaseD, basePIDController::setD);
  
      elbowUpPTuner = new TunableNumber("elbowUpP", kElbowUpP, (a) -> kElbowUpP = a);
      elbowUpITuner = new TunableNumber("elbowUpI", kElbowUpI, (a) -> kElbowUpI = a);
      elbowUpDTuner = new TunableNumber("elbowUpD", kElbowUpD, (a) -> kElbowUpD = a);

      elbowDownPTuner = new TunableNumber("elbowDownP", kElbowDownP, (a) -> kElbowDownP = a);
      elbowDownITuner = new TunableNumber("elbowDownI", kElbowDownI, (a) -> kElbowDownI = a);
      elbowDownDTuner = new TunableNumber("elbowDownD", kElbowDownD, (a) -> kElbowDownD = a);
    } else {
      SmartDashboard.putData("Arm Sim", simArmCanvas);
      elbowFeedForward = new ArmFeedforward(0, 0, 0, 0);
      basePIDController = new ProfiledPIDController(kSimBaseP, 0, 0, kBaseConstraints);
      elbowPIDController = new ProfiledPIDController(kSimElbowP, 0, 0, kElbowConstraints);
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
    basePIDController.setGoal(MathUtil.clamp(basePIDController.getGoal().position+Units.degreesToRadians(bJoystickValue),ArmSubsystem.kMinBaseAngle,ArmSubsystem.kMaxBaseAngle));
    elbowPIDController.setGoal(MathUtil.clamp(elbowPIDController.getGoal().position+Units.degreesToRadians(eJoystickValue),ArmSubsystem.kMinElbowAngle,ArmSubsystem.kMaxElbowAngle));
  }

  public void setBaseReference(double setpoint) {
    basePIDController.setGoal(setpoint);
  }

  public void setElbowReference(double setpoint) {
    if(getElbowAngle() > setpoint){
      elbowPIDController.setP(kElbowDownP);
      elbowPIDController.setI(kElbowDownI);
      elbowPIDController.setD(kElbowDownD);
      elbowUpDownLog.log("Down");
    } else{
      elbowPIDController.setP(kElbowUpP);
      elbowPIDController.setI(kElbowUpI);
      elbowPIDController.setD(kElbowUpD);
      elbowUpDownLog.log("Up");
    }
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
    
    //PID output
    double elbowPIDOutput = elbowPIDController.calculate(getElbowAngle());
    elbowPIDOutputLog.log(elbowPIDOutput);
    //Feedforward output
    double elbowFFOutput = elbowFeedForward.calculate(elbowPIDController.getGoal().position + basePIDController.getGoal().position - Math.PI, 0);
    elbowFFOutputLog.log(elbowFFOutput);
    //Clamp output
    double elbowOutput = MathUtil.clamp(elbowPIDOutput + elbowFFOutput, getElbowAngle() < kMinElbowAngle ? 0 : -kMaxElbowVoltage, getElbowAngle() > kMaxElbowAngle ? 0 : kMaxElbowVoltage);
    elbowOutputLog.log(elbowOutput);

    baseMotor.set(baseOutput);
    elbowMotor.setVoltage(elbowOutput);
  }

  //Cancels commmand, but profPIDcontroller keeps going to next setpoint, before dropping back to where the button was pressed.
  public void cancelCommands() {
    getCurrentCommand().cancel();
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

    desiredXPositionLog.log(x);
    desiredYPositionLog.log(y);
    
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
            ).withName("goToState " + state);
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
      case MIDSCORE:
      case CUBEINTAKE:
        return transitionToState(ArmState.STOWED);
      case HIGHSCORE:
        return goToState(ArmState.HIGHDROP);
      case TRANSITION:
      case CONEINTAKE:
      case HIGHDROP:
        return goToState(ArmState.CUBEINTAKE);
      default:
        return null;
    }
  }

  public ArmState getArmState() {
    return currentState;
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
    SmartDashboard.putData(this);
    // This method will be called once per scheduler run
    calculateCurrentPositions();
    if(DriverStation.isDisabled()){
      holdCurrentPosition();
      basePIDController.reset(getBaseAngle());
      elbowPIDController.reset(getElbowAngle());
    }

    rolloverLog.log(getChooChooAngle() > kMinChooChooAngle && getChooChooAngle() < kMaxChooChooAngle);

    actualBaseAngleLog.log(Units.radiansToDegrees(getBaseAngle()));
    actualChooChooAngleLog.log(Units.radiansToDegrees(getChooChooAngle()));
    desiredBaseGoalLog.log(Units.radiansToDegrees(basePIDController.getGoal().position));
    desiredBaseSetpointLog.log(Units.radiansToDegrees(basePIDController.getSetpoint().position));
    baseOutputLog.log(baseMotor.get());

    actualElbowAngleLog.log(Units.radiansToDegrees(getElbowAngle()));
    desiredElbowGoalLog.log(Units.radiansToDegrees(elbowPIDController.getGoal().position));
    desiredElbowSetpointLog.log(Units.radiansToDegrees(elbowPIDController.getSetpoint().position));
    elbowOutputLog.log(elbowMotor.get());

    actualXPositionLog.log(armXPosition);
    actualYPositionLog.log(armYPosition);
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