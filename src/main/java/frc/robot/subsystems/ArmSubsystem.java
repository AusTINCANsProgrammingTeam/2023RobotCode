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

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.commands.ArmAutoCommand;


public class ArmSubsystem extends SubsystemBase implements AutoCloseable {
  public static enum ArmState{
    STOWED(0,0), //Arm is retracted into the frame perimeter
    INTAKE(0,0), //Arm is in position to intake
    MIDSCORE(0,0), //Arm is in position to score on the mid pole
    HIGHSCORE(0,0); //Arm is in position to score on the high pole

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
  //private TunableNumber elbowPTuner;
  private double kElbowI = 0.036;
  //private TunableNumber elbowITuner;
  private double kElbowD = 0;
  //private TunableNumber elbowDTuner;
  private CANSparkMax motorBaseOne;
  private CANSparkMax motorBaseTwo;
  private CANSparkMax motorElbow;
  private final DutyCycleEncoder motorBaseOneDutyCycleEncoder;
  private final DutyCycleEncoder motorElbowDutyCycleEncoder;
  private double simBaseEncoderPosition;
  private double simElbowEncoderPosition;

  private final RelativeEncoder motorBaseOneRelativeEncoder;
  //private final SparkMaxPIDController basePIDController;
  private final PIDController basePIDController;
  private final PIDController elbowPIDController;
  // SIM VALUES:
  public static final double kBaseOffset = Units.degreesToRadians(-242);
  //-294 -202
  public static final double kElbowOffset = Units.degreesToRadians(380);
  public static final double kBaseGearing = 12.;
  public static final double kElbowGearing = 8.57142857;
  public static final double kBaseArmLength = Units.inchesToMeters(43.5);
  public static final double kElbowArmLength = Units.inchesToMeters(37.5);
  public static final double kMinBAngle = Units.degreesToRadians(45);
  public static final double kMaxBAngle = Units.degreesToRadians(91);
  public static final double kMinEAngle = Units.degreesToRadians(-70);
  public static final double kMaxEAngle = Units.degreesToRadians(89);
  public static final double kBaseArmMass = Units.lbsToKilograms(15); //15
  public static final double kElbowArmMass = Units.lbsToKilograms(15);
  private final Supplier<Double> rJoystick;
  public final double baseArmInertia = SingleJointedArmSim.estimateMOI(kBaseArmLength, kBaseArmMass+kElbowArmMass);
  public final double elbowArmInertia = SingleJointedArmSim.estimateMOI(kElbowArmLength, kElbowArmMass);
  private double simBCurrentAngle;
  private double simECurrentAngle;
  public static boolean controlIsBaseArm;
  public ShuffleboardTab armSimTab = Shuffleboard.getTab("Arm Simulation");
  //Base Arm Values for Simulation
  private GenericEntry simBArmAngle = armSimTab.add("SimBase Arm Angle", 0.0).getEntry();
  private GenericEntry simBEncoderPos = armSimTab.add("SimBase Encoder Angle", 0.0).getEntry();
  private GenericEntry simBOutSet = armSimTab.add("SimBase Output", 0.0).getEntry();
  private GenericEntry simBError = armSimTab.add("SimBase Error", 0.0).getEntry();
  private GenericEntry simBVoltage = armSimTab.add("SimBase Motor Voltage", 0.0).getEntry();
  //Elbow Arm Values for Simulation
  private GenericEntry simEArmAngle = armSimTab.add("SimElbow Arm Angle", 0.0).getEntry();
  private GenericEntry simEEncoderPos = armSimTab.add("SimElbow Encoder Angle", 0.0).getEntry();
  private GenericEntry simEOutSet = armSimTab.add("SimElbow Output", 0.0).getEntry();
  private GenericEntry simEError = armSimTab.add("SimElbow Error", 0.0).getEntry();
  private GenericEntry simEVoltage = armSimTab.add("SimElbow Motor Voltage", 0.0).getEntry();
  //Real arm values
  public ShuffleboardTab armTab = Shuffleboard.getTab("Arm (Real)");
  private GenericEntry baseArmAngle = armTab.add("Base Arm Angle", 0.0).getEntry();
  private GenericEntry baseArmAngleSet = armTab.add("Base Arm Angle Setpoint", 0.0).getEntry();
  private GenericEntry elbowArmAngle = armTab.add("Elbow Arm Angle", 0.0).getEntry();
  private GenericEntry baseArmRelEncoderAngle = armTab.add("Base NEO Angle", 0.0).getEntry();
  private GenericEntry elbowOutput = armTab.add("Elbow Output", 0.0).getEntry();

  private final SingleJointedArmSim baseArmSim = new SingleJointedArmSim(DCMotor.getNEO(2), kBaseGearing, baseArmInertia, kBaseArmLength, kMinBAngle, kMaxBAngle, kBaseArmMass, false);
  private final SingleJointedArmSim elbowArmSim = new SingleJointedArmSim(DCMotor.getNEO(1), kElbowGearing, elbowArmInertia, kElbowArmLength, kMinEAngle, kMaxEAngle, kElbowArmMass, false);

  public ArmSubsystem() {
    controlIsBaseArm = false;
    rJoystick = OI.Operator.getArmRotationSupplier();
    SmartDashboard.putNumber("Elbow P", kElbowP);
    SmartDashboard.putNumber("Elbow I", kElbowI);
    SmartDashboard.putNumber("Elbow D", kElbowD);
    motorBaseOne = MotorController.constructMotor(MotorConfig.ArmBaseMotor1);
    motorBaseTwo = MotorController.constructMotor(MotorConfig.ArmBaseMotor2);
    motorBaseTwo.follow(motorBaseOne);
    motorElbow = MotorController.constructMotor(MotorConfig.ArmElbowMotor);
    //TODO: Place real values for channels
    motorBaseOneDutyCycleEncoder = AbsoluteEncoder.constructREVEncoder(EncoderConfig.ArmBase);
    motorElbowDutyCycleEncoder = AbsoluteEncoder.constructREVEncoder(EncoderConfig.ArmElbow);
    motorBaseOneRelativeEncoder = motorBaseOne.getEncoder();
    basePIDController = new PIDController(kBaseP, kBaseI, kBaseD);
    elbowPIDController = new PIDController(kElbowP, kElbowI, kElbowD);
    //elbowPTuner = new TunableNumber("Elbow P", kElbowP, elbowPIDController::setP);
    //elbowITuner = new TunableNumber("Elbow I", kElbowI, elbowPIDController::setI);
    //elbowDTuner = new TunableNumber("Elbow D", kElbowD, elbowPIDController::setD);
    
  }

  public double getBaseDutyCycleSimAngle() {
    return simBaseEncoderPosition;
  }
  public double getBaseDutyCycleAngle() {
    return AbsoluteEncoder.getPositionRadians(motorBaseOneDutyCycleEncoder);
  }
  public double getElbowDutyCycleSimAngle() {
    return simElbowEncoderPosition;
  }
  public double getElbowDutyCycleAngle() {
    return AbsoluteEncoder.getPositionRadians(motorElbowDutyCycleEncoder);
  }
  public void setBaseRef(double setpoint) {
    motorBaseOne.set(MathUtil.clamp(basePIDController.calculate(getBaseDutyCycleAngle(), setpoint),-1,1));
  }
  public void setElbowRef(double setpoint) {
    motorElbow.set(MathUtil.clamp(elbowPIDController.calculate(getElbowDutyCycleAngle(), setpoint),-1,1));
  }
  public void toggleArmControl() {
    //Toggle control from base arm to elbow arm
    controlIsBaseArm = !controlIsBaseArm;
  }
  
  public void stopArmMotors() {
    motorBaseOne.stopMotor();
    motorElbow.stopMotor();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //motorBaseOneRelativeEncoder.setPosition(getBaseDutyCycleAngle());
    baseArmAngle.setDouble(Units.radiansToDegrees(getBaseDutyCycleAngle()));
    elbowArmAngle.setDouble(Units.radiansToDegrees(getElbowDutyCycleAngle()));
    baseArmRelEncoderAngle.setDouble(Units.radiansToDegrees(motorBaseOneRelativeEncoder.getPosition()));
    //baseArmAngleSet.setDouble((MathUtil.clamp(rJoystick.get(),0,1)*Units.degreesToRadians(45))+Units.degreesToRadians(45));
    baseArmAngleSet.setDouble(rJoystick.get());
    //motorElbow.set(rJoystick.get()*.8);
    elbowOutput.setDouble(motorElbow.getAppliedOutput());
    kElbowP = SmartDashboard.getNumber("Elbow P", kElbowP);
    elbowPIDController.setP(kElbowP);
    kElbowI = SmartDashboard.getNumber("Elbow I", kElbowI);
    elbowPIDController.setI(kElbowI);
    kElbowD = SmartDashboard.getNumber("Elbow D", kElbowD);
    elbowPIDController.setD(kElbowD);
  }

  @Override
  public void simulationPeriodic() {
    //Base arm angle
    double simulationXCoord = 100*Units.inchesToMeters(45.8);
    double simulationYCoord = 100*Units.inchesToMeters(26.9);
    double baseSetpoint = ArmAutoCommand.getBaseAngle(simulationXCoord, simulationYCoord)+Units.degreesToRadians(0);
    double elbowSetpoint = ArmAutoCommand.getElbowAngle(simulationXCoord, simulationYCoord)+Units.degreesToRadians(0);
    double baskElbowPidOut = MathUtil.clamp(basePIDController.calculate(getBaseDutyCycleSimAngle(), baseSetpoint), -1, 1);
    double elbowPidOut = MathUtil.clamp(elbowPIDController.calculate(getElbowDutyCycleSimAngle(), elbowSetpoint), -1, 1);
    baseArmSim.setInputVoltage(baskElbowPidOut * RobotController.getBatteryVoltage());
    elbowArmSim.setInputVoltage(elbowPidOut * RobotController.getBatteryVoltage());
    simBaseEncoderPosition = baseArmSim.getAngleRads();
    simElbowEncoderPosition = elbowArmSim.getAngleRads();
  
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(baseArmSim.getCurrentDrawAmps()+elbowArmSim.getCurrentDrawAmps()));
    
    baseArmSim.update(Robot.kDefaultPeriod); // standard loop time of 20ms
    elbowArmSim.update(Robot.kDefaultPeriod);
    //SB Base Arm Return values
    simBCurrentAngle = Units.radiansToDegrees(baseArmSim.getAngleRads()); //Returns angle in degrees
    simBArmAngle.setDouble(simBCurrentAngle);
    simBEncoderPos.setDouble(Units.radiansToDegrees(getBaseDutyCycleSimAngle()));
    simBEncoderPos.setDouble(Units.radiansToDegrees(getBaseDutyCycleSimAngle()));
    //simBOutSet.setDouble(basePidOut);
    simBOutSet.setDouble(Units.radiansToDegrees(ArmAutoCommand.getBaseAngle(simulationXCoord, simulationYCoord)));
    simBError.setDouble(Units.radiansToDegrees(basePIDController.getPositionError()));
    simBVoltage.setDouble(baseArmSim.getCurrentDrawAmps());
    
    //SB Elbow Arm Return values
    simECurrentAngle = Units.radiansToDegrees(elbowArmSim.getAngleRads()); //Returns angle in degrees
    simEArmAngle.setDouble(simECurrentAngle);
    simEEncoderPos.setDouble(getElbowDutyCycleSimAngle());
    //simEOutSet.setDouble(baskElbowPidOut);
    simEOutSet.setDouble(ArmAutoCommand.getElbowAngle(simulationXCoord, simulationYCoord)*(180/Math.PI));
    simEError.setDouble(Units.radiansToDegrees(elbowPIDController.getPositionError()));
    simEVoltage.setDouble(elbowArmSim.getCurrentDrawAmps());
  } 

  @Override
  public void close() throws Exception {
    // This method will close all device handles used by this object and release any other dynamic memory.
    // Mostly for JUnit tests
    motorBaseOne.close();
    //motorBaseOneDutyCycle.close();
    motorBaseTwo.close();
    motorElbow.close();
    basePIDController.close();
    elbowPIDController.close();
  }
}
