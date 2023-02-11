// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.AbsoluteEncoder;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.AbsoluteEncoder.EncoderConfig;
import frc.robot.hardware.MotorController.MotorConfig;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.system.plant.DCMotor;
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
import frc.robot.Robot;


public class ArmSubsystem extends SubsystemBase implements AutoCloseable {
  public static enum ArmState{
    //FIXME states are not real positions, they are used for sim right now
    STOWED(1.50,0.3), //Arm is retracted into the frame perimeter
    INTAKE(0,0), //Arm is in position to intake
    MIDSCORE(1.40,0.5), //Arm is in position to score on the mid pole
    HIGHSCORE(1.30,0.6); //Arm is in position to score on the high pole

    private double x; //Position relative to the base of the arm, in meters
    private double y; //Positon above the carpet, in meters
    
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

  // FIXME using PWMSparkMax because CANSparkMax doesn't have an equivalent simulation class
  // FIXME https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html
  // May limit how much we can do in terms of JUnit tests
  //Base arm PID values
  private double kBaseP = 1;
  private double kBaseI = 0.036;
  private double kBaseD = 0;
  //Elbow arm PID values
  private double kElbowP = 1;
  private double kElbowI = 0.036;
  private double kElbowD = 0;
  //Sim PID values
  private double kSimBaseP = 0.1;
  private double kSimBaseI = 0.0;
  private double kSimBaseD = 0.0;
  private double kSimElbowP = 0.1;
  private double kSimElbowI = 0.0;
  private double kSimElbowD = 0.0;

  private double desiredBAngle;
  private double desiredEAngle;

  private CANSparkMax baseMotor;
  private CANSparkMax baseMotor2;
  private CANSparkMax elbowMotor;

  private final DutyCycleEncoder baseAbsoluteEncoder;
  private final DutyCycleEncoder elbowAbsoluteEncoder;

  private double simBaseEncoderPosition;
  private double simElbowEncoderPosition;

  private final PIDController basePIDController;
  private final PIDController elbowPIDController;
  public static final double kBaseGearing = 12;
  public static final double kElbowGearing = 8.57142857;
  public static final double kBaseArmLength = Units.inchesToMeters(43.5);
  public static final double kElbowArmLength = Units.inchesToMeters(37.5);
  public static final double kMinBAngle = Units.degreesToRadians(30);
  public static final double kMaxBAngle = Units.degreesToRadians(91);
  public static final double kMinEAngle = Units.degreesToRadians(-70);
  public static final double kMaxEAngle = Units.degreesToRadians(89);
  public static final double kBaseArmMass = 15;
  public static final double kElbowArmMass = 15;
  public final double baseArmInertia = SingleJointedArmSim.estimateMOI(kBaseArmLength, kBaseArmMass);
  public final double elbowArmInertia = SingleJointedArmSim.estimateMOI(kElbowArmLength, kElbowArmMass);
  public static boolean controlIsBaseArm;
  public ShuffleboardTab armSimTab = Shuffleboard.getTab("Arm Simulation");
  //Base Arm Values for Simulation
  private GenericEntry simBArmAngle = armSimTab.add("SimBase Arm Angle", 0.0).getEntry();
  private GenericEntry simBEncoderPos = armSimTab.add("SimBase Encoder Angle", 0.0).getEntry();
  private GenericEntry simBOutSet = armSimTab.add("SimBase Output", 0.0).getEntry();
  private GenericEntry simArmStateX = armSimTab.add("Arm State X", 0.0).getEntry();
  private GenericEntry simArmStateY = armSimTab.add("Arm State Y", 0.0).getEntry();
  //Elbow Arm Values for Simulation
  private GenericEntry simEArmAngle = armSimTab.add("SimElbow Arm Angle", 0.0).getEntry();
  private GenericEntry simEOutSet = armSimTab.add("SimElbow Output", 0.0).getEntry();
  //Real arm values
  public ShuffleboardTab armTab = Shuffleboard.getTab("Arm (Real)");
  private GenericEntry baseArmAngle = armTab.add("Base Arm Angle", 0.0).getEntry();
  private GenericEntry baseArmAngleSetpoint = armTab.add("Base Angle Setpoint", 0.0).getEntry();
  private GenericEntry elbowArmAngleSetpoint = armTab.add("Elbow Angle Setpoint", 0.0).getEntry();
  private GenericEntry elbowArmAngle = armTab.add("Elbow Arm Angle", 0.0).getEntry();
  private GenericEntry elbowOutput = armTab.add("Elbow Output", 0.0).getEntry();
  private GenericEntry armXPosition = armTab.add("Arm X Position", 0.0).getEntry();
  private GenericEntry armYPosition = armTab.add("Arm Y Position", 0.0).getEntry();

  private final SingleJointedArmSim baseArmSim = new SingleJointedArmSim(DCMotor.getNEO(1), kBaseGearing, baseArmInertia, kBaseArmLength, kMinBAngle, kMaxBAngle, kBaseArmMass, false);
  private final SingleJointedArmSim elbowArmSim = new SingleJointedArmSim(DCMotor.getNEO(1), kElbowGearing, elbowArmInertia, kElbowArmLength, kMinEAngle, kMaxEAngle, kElbowArmMass, false);

  //The "canvas" that both arms are drawn on
  Mechanism2d simArmCanvas = new Mechanism2d(7,7);
  //Where the base arm begins
  MechanismRoot2d baseArmSimRoot = simArmCanvas.getRoot("Base Arm Root", 0, 0);
  //Sets the base arm to the root, then the elbow arm to the end of the base arm
  MechanismLigament2d baseArmSimV = baseArmSimRoot.append(new MechanismLigament2d("Base Arm", kBaseArmLength*3, baseArmSim.getAngleRads()));
  MechanismLigament2d elbowArmSimV = baseArmSimV.append(new MechanismLigament2d("Elbow Arm", kElbowArmLength*3, elbowArmSim.getAngleRads()));
  

  public ArmSubsystem() {
    SmartDashboard.putData("Arm Sim", simArmCanvas);
    controlIsBaseArm = true;
    SmartDashboard.putNumber("Elbow P", kElbowP);
    SmartDashboard.putNumber("Elbow I", kElbowI);
    SmartDashboard.putNumber("Elbow D", kElbowD);
    baseMotor = MotorController.constructMotor(MotorConfig.ArmBase1);
    baseMotor2 = MotorController.constructMotor(MotorConfig.ArmBase2);
    baseMotor2.follow(baseMotor);
    elbowMotor = MotorController.constructMotor(MotorConfig.ArmElbow);
    baseAbsoluteEncoder = AbsoluteEncoder.constructREVEncoder(EncoderConfig.ArmBase);
    elbowAbsoluteEncoder = AbsoluteEncoder.constructREVEncoder(EncoderConfig.ArmElbow);
    if(Robot.isSimulation()) {
      basePIDController = new PIDController(kSimBaseP, kSimBaseI, kSimBaseD);
      elbowPIDController = new PIDController(kSimElbowP, kSimElbowI, kSimElbowD);
    } else {
      basePIDController = new PIDController(kBaseP, kBaseI, kBaseD);
      elbowPIDController = new PIDController(kElbowP, kElbowI, kElbowD);
    }
    setState(ArmState.STOWED);
  }

  public double getBaseAngle() {
    return Robot.isSimulation() ? simBaseEncoderPosition : AbsoluteEncoder.getPositionRadians(baseAbsoluteEncoder);
  }

  public void setBase(double speed){
    baseMotor.set(speed);
    SmartDashboard.putNumber("Applied Output", baseMotor.getAppliedOutput());
    SmartDashboard.putNumber("Current", baseMotor.getOutputCurrent());
    
  }

  public double getElbowAngle() {
    return Robot.isSimulation() ? simElbowEncoderPosition : AbsoluteEncoder.getPositionRadians(elbowAbsoluteEncoder);
  }

  public void setBaseReference(double setpoint) {
    basePIDController.setSetpoint(setpoint);
    
  }
  public void updateMotors() {
    baseMotor.set(MathUtil.clamp(basePIDController.calculate(getBaseAngle()),-1,1));
    elbowMotor.set(MathUtil.clamp(elbowPIDController.calculate(getElbowAngle()),-1,1));
  }
  public void updateSimMotors() {
    baseArmSim.setInputVoltage((MathUtil.clamp(basePIDController.calculate(getBaseAngle()),-1,1)) * RobotController.getBatteryVoltage());
    elbowArmSim.setInputVoltage((MathUtil.clamp(elbowPIDController.calculate(getElbowAngle()),-1,1)) * RobotController.getBatteryVoltage());
  }

  public void setElbowReference(double setpoint) {
    elbowPIDController.setSetpoint(setpoint);
  }

  //Angle finding methods

  public static double convertToPrelimAngle(double x, double y) {
    return (
      -1*Math.acos(
        ( (x*x) + (y*y) - (kBaseArmLength*kBaseArmLength) - (kElbowArmLength*kElbowArmLength) )/(2*kBaseArmLength*kElbowArmLength)
      )
    );
  }

  public static double convertToBaseAngle(double x, double y) {
    double pAngle = convertToPrelimAngle(x, y);
    return (
      Math.atan(y/x)-Math.atan((kElbowArmLength*Math.sin(pAngle) )/( (kBaseArmLength+(kElbowArmLength*Math.cos(pAngle))))
      )
    );
  }

  public static double convertToElbowAngle(double x, double y) {
    return Units.degreesToRadians(90) + convertToPrelimAngle(x, y) + convertToBaseAngle(x, y);
  }

  public void setState(ArmState state){
    double desiredBaseAngle = convertToBaseAngle(state.x,state.y);
    double desiredElbowAngle = convertToElbowAngle(state.x,state.y);
    desiredBAngle = desiredBaseAngle;
    desiredEAngle = desiredElbowAngle;
    armXPosition.setDouble(state.getX());
    armYPosition.setDouble(state.getY());
    simArmStateX.setDouble(state.getX());
    simArmStateY.setDouble(state.getY());
    
    setBaseReference(desiredBaseAngle);
    setElbowReference(desiredElbowAngle);
  }

  public void toggleArmControl() {
    //Toggle control from base arm to elbow arm
    controlIsBaseArm = !controlIsBaseArm;
  }
  
  public void stop() {
    baseMotor.stopMotor();
    elbowMotor.stopMotor();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateMotors();

    //Shuffleboard + Smartdashboard values 
    baseArmAngle.setDouble(Units.radiansToDegrees(getBaseAngle()));
    elbowArmAngle.setDouble(Units.radiansToDegrees(getElbowAngle()));
    baseArmAngleSetpoint.setDouble(Units.radiansToDegrees(basePIDController.getSetpoint()));
    elbowArmAngleSetpoint.setDouble(Units.radiansToDegrees(elbowPIDController.getSetpoint()));
    elbowOutput.setDouble(elbowMotor.getAppliedOutput());
    if(!Robot.isSimulation()) {
      kElbowP = SmartDashboard.getNumber("Elbow P", kElbowP);
      elbowPIDController.setP(kElbowP);
      kElbowI = SmartDashboard.getNumber("Elbow I", kElbowI);
      elbowPIDController.setI(kElbowI);
      kElbowD = SmartDashboard.getNumber("Elbow D", kElbowD);
      elbowPIDController.setD(kElbowD);
    }
  }

  @Override
  public void simulationPeriodic() {
    updateSimMotors();
    simBaseEncoderPosition = baseArmSim.getAngleRads();
    simElbowEncoderPosition = elbowArmSim.getAngleRads();
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(baseArmSim.getCurrentDrawAmps()+elbowArmSim.getCurrentDrawAmps()));
    baseArmSimV.setAngle(Units.radiansToDegrees(baseArmSim.getAngleRads()));
    elbowArmSimV.setAngle(Units.radiansToDegrees(elbowArmSim.getAngleRads())-baseArmSimV.getAngle()-90);
    baseArmSim.update(Robot.kDefaultPeriod); // standard loop time of 20ms
    elbowArmSim.update(Robot.kDefaultPeriod);

    //Shuffleboard values
    simBArmAngle.setDouble(Units.radiansToDegrees(baseArmSim.getAngleRads()));
    simBEncoderPos.setDouble(Units.radiansToDegrees(getBaseAngle()));
    simBOutSet.setDouble(Units.radiansToDegrees(desiredBAngle));
    simEArmAngle.setDouble(Units.radiansToDegrees(elbowArmSim.getAngleRads()));
    simEOutSet.setDouble(Units.radiansToDegrees(desiredEAngle));
  }

  @Override
  public void close() throws Exception {
    // This method will close all device handles used by this object and release any other dynamic memory.
    // Mostly for JUnit tests
    baseMotor.close();
    baseAbsoluteEncoder.close();
    elbowMotor.close();
    basePIDController.close();
    elbowPIDController.close();
  }
}
