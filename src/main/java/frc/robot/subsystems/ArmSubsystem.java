// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.commands.ArmAutoCommand;


public class ArmSubsystem extends SubsystemBase implements AutoCloseable {

  // FIXME using PWMSparkMax because CANSparkMax doesn't have an equivalent simulation class
  // FIXME https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html
  // May limit how much we can do in terms of JUnit tests
  //Base arm PID values
  private double BP = 1;
  private double BI = 0;
  private double BD = 0.1;
  //Elbow arm PID values
  private double EP = 0.025;
  private double EI = 0.02;
  private double ED = 0.175;
  private CANSparkMax motorBaseOne;
  private CANSparkMax motorBaseTwo;
  private CANSparkMax motorElbow;
  private final Encoder motorBaseOneEncoder;
  private final EncoderSim motorBaseOneEncoderSim;
  private final Encoder motorElbowEncoder;
  private final EncoderSim motorElbowEncoderSim;
  private final PIDController basePIDController;
  private final PIDController elbowPIDController;
  private ShuffleboardTab configTab = Shuffleboard.getTab("Arm");
  //Base Arm Values for Simulation
  private GenericEntry simBArmAngle = configTab.add("SimBase Arm Angle", 0.0).getEntry();
  private GenericEntry simBEncoderPos = configTab.add("SimBase Encoder Angle", 0.0).getEntry();
  private GenericEntry simBOutSet = configTab.add("SimBase Output", 0.0).getEntry();
  private GenericEntry simBError = configTab.add("SimBase Error", 0.0).getEntry();
  private GenericEntry simBVoltage = configTab.add("SimBase Motor Voltage", 0.0).getEntry();
  //Elbow Arm Values for Simulation
  private GenericEntry simEArmAngle = configTab.add("SimElbow Arm Angle", 0.0).getEntry();
  private GenericEntry simEEncoderPos = configTab.add("SimElbow Encoder Angle", 0.0).getEntry();
  private GenericEntry simEOutSet = configTab.add("SimElbow Output", 0.0).getEntry();
  private GenericEntry simEError = configTab.add("SimElbow Error", 0.0).getEntry();
  private GenericEntry simEVoltage = configTab.add("SimElbow Motor Voltage", 0.0).getEntry();
  

  // SIM VALUES:
  private double baseGearing = 15.;
  private double elbowGearing = 8.57142857;
  public static double baseArmLength = Units.inchesToMeters(43.5);
  public static double elbowArmLength = Units.inchesToMeters(37.5);
  private double minBAngle = Units.degreesToRadians(45);
  private double maxBAngle = Units.degreesToRadians(91);
  private double minEAngle = Units.degreesToRadians(-70);
  private double maxEAngle = Units.degreesToRadians(89);
  private double baseArmMass = Units.lbsToKilograms(15); //15
  private double elbowArmMass = Units.lbsToKilograms(15);
  private final double baseArmInertia = SingleJointedArmSim.estimateMOI(baseArmLength, baseArmMass);
  private final double elbowArmInertia = SingleJointedArmSim.estimateMOI(elbowArmLength, elbowArmMass);
  private double simBCurrentAngle;
  private double simECurrentAngle;

  private final SingleJointedArmSim baseArmSim = new SingleJointedArmSim(DCMotor.getNEO(2), baseGearing, baseArmInertia, baseArmLength, minBAngle, maxBAngle, baseArmMass, false);
  private final SingleJointedArmSim elbowArmSim = new SingleJointedArmSim(DCMotor.getNEO(1), elbowGearing, elbowArmInertia, elbowArmLength, minEAngle, maxEAngle, elbowArmMass, false);

  public ArmSubsystem() {
    motorBaseOne = MotorController.constructMotor(MotorConfig.ArmBaseMotor1);
    motorBaseTwo = MotorController.constructMotor(MotorConfig.ArmBaseMotor2);
    motorBaseTwo.follow(motorBaseOne);
    motorElbow = MotorController.constructMotor(MotorConfig.ArmElbowMotor);
    //TODO: Place real values for channels
    motorBaseOneEncoder = new Encoder(0,1);
    motorElbowEncoder = new Encoder(2,3);
    motorBaseOneEncoder.setDistancePerPulse(1./baseGearing); //Amount of distance per rotation of the motor, basically a gear ratio measurement
    motorElbowEncoder.setDistancePerPulse(1./elbowGearing);
    motorBaseOneEncoderSim = new EncoderSim(motorBaseOneEncoder);
    motorElbowEncoderSim = new EncoderSim(motorElbowEncoder);
    basePIDController = new PIDController(BP, BI, BD);
    elbowPIDController = new PIDController(EP, EI, ED);
  }

  @Override
  public void periodic() {  
    // This method will be called once per scheduler run
  }

  public void setBaseRef(double setpoint) {
    motorBaseOne.set(basePIDController.calculate(motorBaseOneEncoder.getDistance(), setpoint));
  }
  public double getBaseAngle() {
    return motorBaseOneEncoder.getDistance() * (Math.PI*2);
  }

   public void setElbowRef(double setpoint) {
    motorElbow.set(elbowPIDController.calculate(motorElbowEncoder.getDistance(), setpoint));
  }
  public double getElbowAngle() {
    return motorElbowEncoder.getDistance() * (Math.PI*2);
  }

  @Override
  public void simulationPeriodic() {
    //Base arm angle
    double simulationXCoord = 100*Units.inchesToMeters(45.8);
    double simulationYCoord = 100*Units.inchesToMeters(26.9);
    double baseSetpoint = ArmAutoCommand.getBaseAngle(simulationXCoord, simulationYCoord);
    double elbowSetpoint = ArmAutoCommand.getElbowAngle(simulationXCoord, simulationYCoord);
    double basePidOut = basePIDController.calculate(motorBaseOneEncoderSim.getDistance(), baseSetpoint);
    double elbowPidOut = elbowPIDController.calculate(motorElbowEncoderSim.getDistance(), elbowSetpoint);
    baseArmSim.setInputVoltage(basePidOut * RobotController.getBatteryVoltage());
    elbowArmSim.setInputVoltage(elbowPidOut * RobotController.getBatteryVoltage());
    motorBaseOneEncoderSim.setDistance(baseArmSim.getAngleRads());
    motorElbowEncoderSim.setDistance(elbowArmSim.getAngleRads());
  
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(baseArmSim.getCurrentDrawAmps()+elbowArmSim.getCurrentDrawAmps()));
    
    baseArmSim.update(0.02); // standard loop time of 20ms
    elbowArmSim.update(0.02);
    //SB Base Arm Return values
    simBCurrentAngle = Units.radiansToDegrees(baseArmSim.getAngleRads()); //Returns angle in degrees
    simBArmAngle.setDouble(simBCurrentAngle);
    simBEncoderPos.setDouble(motorBaseOneEncoderSim.getDistance()*(180/Math.PI));
    simBOutSet.setDouble(basePidOut);
    //simBOutSet.setDouble(ArmAutoCommand.getBaseAngle(simulationXCoord, simulationYCoord)*(180/Math.PI));
    simBError.setDouble(basePIDController.getPositionError()*(180/Math.PI));
    simBVoltage.setDouble(baseArmSim.getCurrentDrawAmps());
    
    //SB Elbow Arm Return values
    simECurrentAngle = Units.radiansToDegrees(elbowArmSim.getAngleRads()); //Returns angle in degrees
    simEArmAngle.setDouble(simECurrentAngle);
    simEEncoderPos.setDouble(motorElbowEncoderSim.getDistance()*(180/Math.PI));
    //simEOutSet.setDouble(basePidOut);
    simEOutSet.setDouble(ArmAutoCommand.getElbowAngle(simulationXCoord, simulationYCoord)*(180/Math.PI));
    simEError.setDouble(elbowPIDController.getPositionError()*(180/Math.PI));
    simEVoltage.setDouble(elbowArmSim.getCurrentDrawAmps());
  } 

  @Override
  public void close() throws Exception {
    // This method will close all device handles used by this object and release any other dynamic memory.
    // Mostly for JUnit tests
    motorBaseOne.close(); 
  }
}
