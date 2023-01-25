// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.RelativeEncoderSim;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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
  private double P = 0.01;
  private double I = 0;
  private double D = 0.6;
  private CANSparkMax motorBaseOne;
  private CANSparkMax motorBaseTwo;
  private CANSparkMax motorElbow;
  private final Encoder motorBaseOneEncoder;
  //private final RelativeEncoder motorBaseOneEncoder;
  //private final RelativeEncoderSim motorBaseOneEncoderSim;
  private final EncoderSim motorBaseOneEncoderSim;
  private final RelativeEncoder motorElbowEncoder;
  private final RelativeEncoderSim motorElbowEncoderSim;
  private final PIDController basePIDController;
  private final PIDController elbowPIDController;
  private ShuffleboardTab configTab = Shuffleboard.getTab("Arm");
  private GenericEntry armAngleSim = configTab.add("Simulation Arm Angle", 0.0).getEntry();
  private GenericEntry simEncoderPos = configTab.add("Simulation Encoder Angle", 0.0).getEntry();
  private GenericEntry simOutSet = configTab.add("Simulation Output", 0.0).getEntry();
  private GenericEntry simError = configTab.add("Simulation Error", 0.0).getEntry();
  private GenericEntry simVoltage = configTab.add("Simulation Motor Voltage", 0.0).getEntry();

  // SIM VALUES:
  private double gearing = 1;
  private double baseArmLength = Units.inchesToMeters(37.5);
  private double elbowArmLength = Units.inchesToMeters(37.5);
  private double minAngle = Units.degreesToRadians(-160);
  private double maxAngle = Units.degreesToRadians(160);
  private double baseArmMass = Units.lbsToKilograms(15); //15
  private double elbowArmMass = Units.lbsToKilograms(15);
  private final double baseArmInertia = SingleJointedArmSim.estimateMOI(baseArmLength, baseArmMass);
  private final double elbowArmInertia = SingleJointedArmSim.estimateMOI(elbowArmLength, elbowArmMass);
  private double simCurrentAngle;

  private final SingleJointedArmSim baseArmSim = new SingleJointedArmSim(DCMotor.getNEO(1), gearing, baseArmInertia, baseArmLength, minAngle, maxAngle, baseArmMass, false);
  private final SingleJointedArmSim elbowArmSim = new SingleJointedArmSim(DCMotor.getNEO(1), gearing, elbowArmInertia, elbowArmLength, minAngle, maxAngle, elbowArmMass, false);

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    motorBaseOne = MotorController.constructMotor(MotorConfig.ArmBaseMotor1);
    motorBaseTwo = MotorController.constructMotor(MotorConfig.ArmBaseMotor2);
    motorBaseTwo.follow(motorBaseOne);
    motorElbow = MotorController.constructMotor(MotorConfig.ArmElbowMotor);
    
    //Encoders are not used right now, might be implemented later
    //motorBaseOneEncoderSim = new RelativeEncoderSim(motorBaseOne);
    motorElbowEncoderSim = new RelativeEncoderSim(motorElbow);
    //motorBaseOneEncoder = this.motorBaseOne.getEncoder(); 
    //TODO: Place real values for channels
    motorBaseOneEncoder = new Encoder(0,1);
    motorBaseOneEncoderSim = new EncoderSim(motorBaseOneEncoder);
    motorElbowEncoder = this.motorElbow.getEncoder(); 
    basePIDController = new PIDController(P, I, D);
    elbowPIDController = new PIDController(P, I, D);
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
    motorElbow.set(elbowPIDController.calculate(motorElbowEncoder.getPosition(), setpoint));
  }

  @Override
  public void simulationPeriodic() {
    //Base arm angle
    double baseSetpoint = ArmAutoCommand.getBaseAngle(76, 88);
    double elbowSetpoint = ArmAutoCommand.getElbowAngle(76, 88);
    double basePidOut = basePIDController.calculate(motorBaseOneEncoderSim.getDistance()*(2*Math.PI), baseSetpoint);
    double elbowPidOut = elbowPIDController.calculate(motorElbowEncoderSim.getPosition()*(2*Math.PI), elbowSetpoint);
    //motorBaseOne.set(basePidOut);
    //motorBaseOne.set(0);
    motorElbow.set(elbowPidOut);
    //baseArmSim.setInputVoltage(basePidOut * RobotController.getBatteryVoltage());
    baseArmSim.setInputVoltage(0.1);
    elbowArmSim.setInputVoltage(elbowPidOut * RobotController.getBatteryVoltage());
  
    //motorBaseOneEncoderSim.setDistance((Units.radiansToDegrees(baseArmSim.getAngleRads()) * gearing)/360);
    //motorBaseOneEncoderSim.setDistance(2);
    //motorBaseOneEncoderSim.setDistance(baseArmSim.getAngleRads()/(2*Math.PI));
    //motorBaseOneEncoderSim.setDistance(0.1);
    motorElbowEncoderSim.setPosition(elbowArmSim.getAngleRads()/(2*Math.PI) * gearing);
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(baseArmSim.getCurrentDrawAmps()));
    
    baseArmSim.update(0.02); // standard loop time of 20ms
    //SB Return values
    simCurrentAngle = Units.radiansToDegrees(baseArmSim.getAngleRads()); //Returns angle in degrees
    armAngleSim.setDouble(simCurrentAngle);
    //simEncoderPos.setDouble(motorBaseOneEncoderSim.getDistance()*(2*Math.PI)*(180/Math.PI));


    motorBaseOneEncoderSim.setDistance(1.1);
    simEncoderPos.setDouble(motorBaseOneEncoderSim.getDistance());
    //simEncoderPos.setDouble(0.1);

    //System.out.println(motorBaseOneEncoderSim.getDistance());



    //simOutSet.setDouble(ArmAutoCommand.getPrelimAngle(76,88)*(180/Math.PI));
    simOutSet.setDouble(basePidOut);
    simError.setDouble(basePIDController.getPositionError()*(180/Math.PI));
    simVoltage.setDouble(baseArmSim.getCurrentDrawAmps());
  } 

  @Override
  public void close() throws Exception {
    // This method will close all device handles used by this object and release any other dynamic memory.
    // Mostly for JUnit tests
    motorBaseOne.close(); 
  }
}
