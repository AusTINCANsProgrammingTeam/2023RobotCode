// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DutyCycleSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.commands.ArmAutoCommand;


public class ArmSubsystem extends SubsystemBase implements AutoCloseable {

  // FIXME using PWMSparkMax because CANSparkMax doesn't have an equivalent simulation class
  // FIXME https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html
  // May limit how much we can do in terms of JUnit tests
  //Base arm PID values
  private double kBaskElbowP = 1;
  private double kBaskElbowI = 0;
  private double kBaskElbowD = 0.1;
  //Elbow arm PID values
  private double kElbowP = 0.05;
  private double kElbowI = 0;
  private double kElbowD = 0.2;
  private CANSparkMax motorBaseOne;
  private CANSparkMax motorBaseTwo;
  private CANSparkMax motorElbow;
  //private final Encoder motorBaseOneEncoder;
  private final DutyCycle motorBaseOneDutyCycle;
  public static final int kMotorBaseOneDutyCycleChannel = 0;
  private final DutyCycleSim motorBaseOneDutyCycleSim;
  private final DutyCycle motorElbowDutyCycle;
  public static final int kMotorElbowDutyCycleChannel = 1;
  private final DutyCycleSim motorElbowDutyCycleSim;
  //private final EncoderSim motorBaseOneEncoderSim;
  //private final Encoder motorElbowEncoder;
  //private final EncoderSim motorElbowEncoderSim;
  //public static final int kMotorBaseOneEncoderChannelA = 0;
  //public static final int kMotorBaseOneEncoderChannelB = 1;
  //public static final int kMotorElbowEncoderChannelA = 2;
  //public static final int kMotorElbowEncoderChannelB = 3;
  //private final SparkMaxPIDController basePIDController;
  private final PIDController basePIDController;
  private final PIDController elbowPIDController;
  // SIM VALUES:
  public static final double kBaseOffset = 0;
  public static final double kElbowOffset = 0;
  public static final double kBaseGearing = 15.;
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
  

  

  private final SingleJointedArmSim baseArmSim = new SingleJointedArmSim(DCMotor.getNEO(2), kBaseGearing, baseArmInertia, kBaseArmLength, kMinBAngle, kMaxBAngle, kBaseArmMass, false);
  private final SingleJointedArmSim elbowArmSim = new SingleJointedArmSim(DCMotor.getNEO(1), kElbowGearing, elbowArmInertia, kElbowArmLength, kMinEAngle, kMaxEAngle, kElbowArmMass, false);

  public ArmSubsystem() {
    controlIsBaseArm = true;
    rJoystick = OI.Driver.getArmRotationSupplier();
    motorBaseOne = MotorController.constructMotor(MotorConfig.ArmBaseMotor1);
    motorBaseTwo = MotorController.constructMotor(MotorConfig.ArmBaseMotor2);
    motorBaseTwo.follow(motorBaseOne);
    motorElbow = MotorController.constructMotor(MotorConfig.ArmElbowMotor);
    //TODO: Place real values for channels
    //motorBaseOneEncoder = new Encoder(kMotorBaseOneEncoderChannelA,kMotorBaseOneEncoderChannelB);
    motorBaseOneDutyCycle = new DutyCycle(new DigitalInput(kMotorBaseOneDutyCycleChannel));
    motorElbowDutyCycle = new DutyCycle(new DigitalInput(kMotorElbowDutyCycleChannel));
    //motorElbowEncoder = new Encoder(kMotorElbowEncoderChannelA,kMotorElbowEncoderChannelB);
    //motorBaseOneEncoder.setDistancePerPulse((1./kBaseGearing)*(Math.PI*2)); //Amount of distance per rotation of the motor, basically a gear ratio measurement
    //motorElbowEncoder.setDistancePerPulse((1./kElbowGearing)*(Math.PI*2));
    //motorBaseOneEncoderSim = new EncoderSim(motorBaseOneEncoder);
    motorBaseOneDutyCycleSim = new DutyCycleSim(motorBaseOneDutyCycle);
    motorElbowDutyCycleSim = new DutyCycleSim(motorElbowDutyCycle);
    //motorBaseOneEncoderSim.setDistance(Units.degreesToRadians(kMinBAngle));
    if(Robot.isSimulation()) {
      motorBaseOneDutyCycleSim.setOutput((kMinBAngle/(Math.PI*2)));
      motorElbowDutyCycleSim.setOutput((kMinEAngle/(Math.PI*2)));
    }
    //motorElbowEncoderSim = new EncoderSim(motorElbowEncoder);
    //motorElbowEncoderSim.setDistance(Units.degreesToRadians(kMinEAngle));
    basePIDController = new PIDController(kBaskElbowP, kBaskElbowI, kBaskElbowD);
    //basePIDController = motorBaseOne.getPIDController();
    elbowPIDController = new PIDController(kElbowP, kElbowI, kElbowD);
  }

  public double getBaseDutyCycleAngle() {
    return motorBaseOneDutyCycleSim.getOutput()*(Math.PI*2)+kBaseOffset;
  }
  public double getElbowDutyCycleAngle() {
    return motorElbowDutyCycleSim.getOutput()*(Math.PI*2)+kElbowOffset;
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
    baseArmAngle.setDouble(getBaseDutyCycleAngle());
    elbowArmAngle.setDouble(getElbowDutyCycleAngle());
    baseArmAngleSet.setDouble((MathUtil.clamp(rJoystick.get(),0,1)*Units.degreesToRadians(45))+Units.degreesToRadians(45));
  }

  @Override
  public void simulationPeriodic() {
    //Base arm angle
    double simulationXCoord = 100*Units.inchesToMeters(45.8);
    double simulationYCoord = 100*Units.inchesToMeters(26.9);
    double baseSetpoint = ArmAutoCommand.getBaseAngle(simulationXCoord, simulationYCoord)+Units.degreesToRadians(0);
    double elbowSetpoint = ArmAutoCommand.getElbowAngle(simulationXCoord, simulationYCoord)+Units.degreesToRadians(0);
    double baskElbowPidOut = MathUtil.clamp(basePIDController.calculate(getBaseDutyCycleAngle(), baseSetpoint), -1, 1);
    double elbowPidOut = MathUtil.clamp(elbowPIDController.calculate(getElbowDutyCycleAngle(), elbowSetpoint), -1, 1);
    baseArmSim.setInputVoltage(baskElbowPidOut * RobotController.getBatteryVoltage());
    elbowArmSim.setInputVoltage(elbowPidOut * RobotController.getBatteryVoltage());
    //motorBaseOneEncoderSim.setDistance(baseArmSim.getAngleRads());
    motorBaseOneDutyCycleSim.setOutput((baseArmSim.getAngleRads()/(Math.PI*2)));
    motorElbowDutyCycleSim.setOutput((elbowArmSim.getAngleRads()/(Math.PI*2)));
    //motorElbowEncoderSim.setDistance(elbowArmSim.getAngleRads());
  
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(baseArmSim.getCurrentDrawAmps()+elbowArmSim.getCurrentDrawAmps()));
    
    baseArmSim.update(Robot.kDefaultPeriod); // standard loop time of 20ms
    elbowArmSim.update(Robot.kDefaultPeriod);
    //SB Base Arm Return values
    simBCurrentAngle = Units.radiansToDegrees(baseArmSim.getAngleRads()); //Returns angle in degrees
    simBArmAngle.setDouble(simBCurrentAngle);
    simBEncoderPos.setDouble(Units.radiansToDegrees(getBaseDutyCycleAngle()));
    simBEncoderPos.setDouble(Units.radiansToDegrees(getBaseDutyCycleAngle()));
    //simBOutSet.setDouble(basePidOut);
    simBOutSet.setDouble(Units.radiansToDegrees(ArmAutoCommand.getBaseAngle(simulationXCoord, simulationYCoord)));
    simBError.setDouble(Units.radiansToDegrees(basePIDController.getPositionError()));
    simBVoltage.setDouble(baseArmSim.getCurrentDrawAmps());
    
    //SB Elbow Arm Return values
    simECurrentAngle = Units.radiansToDegrees(elbowArmSim.getAngleRads()); //Returns angle in degrees
    simEArmAngle.setDouble(simECurrentAngle);
    simEEncoderPos.setDouble(getElbowDutyCycleAngle());
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
    motorBaseOneDutyCycle.close();
    motorBaseTwo.close();
    motorElbow.close();
    motorElbowDutyCycle.close();
    basePIDController.close();
    elbowPIDController.close();
  }
}
