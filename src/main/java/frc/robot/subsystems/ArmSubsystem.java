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
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;


public class ArmSubsystem extends SubsystemBase implements AutoCloseable {

  // FIXME using PWMSparkMax because CANSparkMax doesn't have an equivalent simulation class
  // May limit how much we can do in terms of JUnit tests
  private double P = 0.1;
  private double I = 0.1;
  private double D = 0;
  private CANSparkMax motorBase;
  private CANSparkMax motorElbow;
  private final RelativeEncoder motorBaseEncoder;
  private final RelativeEncoder motorElbowEncoder;
  private final PIDController basePIDController;
  private final PIDController elbowPIDController;
  private ShuffleboardTab configTab = Shuffleboard.getTab("Arm");
  private GenericEntry armAngleSim = configTab.add("Simulation Arm Angle", 0.0).getEntry();

  // SIM VALUES:
  private double gearing = 1;
  private double baseArmLength = Units.inchesToMeters(40);
  private double minAngle = Units.degreesToRadians(-160);
  private double maxAngle = Units.degreesToRadians(160);
  private double baseArmMass = Units.lbsToKilograms(15);
  private final double baseArmInertia = SingleJointedArmSim.estimateMOI(baseArmLength, baseArmMass);
  private double simCurrentAngle;

  private final SingleJointedArmSim baseArmSim = new SingleJointedArmSim(DCMotor.getNEO(1), gearing, baseArmInertia, baseArmLength, minAngle, maxAngle, baseArmMass, false);

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    motorBase = MotorController.constructMotor(MotorConfig.ArmBaseMotor);
    motorElbow = MotorController.constructMotor(MotorConfig.ArmElbowMotor);
    
    //Encoders are not used right now, might be implemented later
    motorBaseEncoder = this.motorBase.getEncoder(); 
    motorElbowEncoder = this.motorElbow.getEncoder(); 
    basePIDController = new PIDController(P, I, D);
    elbowPIDController = new PIDController(P, I, D);
  }

  @Override
  public void periodic() {  
    // This method will be called once per scheduler run
  }

  public void setBaseRef(double setpoint) {
    motorBase.set(basePIDController.calculate(motorBaseEncoder.getPosition(), setpoint));
  }

   public void setElbowRef(double setpoint) {
    motorElbow.set(elbowPIDController.calculate(motorElbowEncoder.getPosition(), setpoint));
  }

  @Override
  public void simulationPeriodic() {
    baseArmSim.setInputVoltage(basePIDController.calculate(motorBaseEncoder.getPosition(), Units.degreesToRadians(5)));
    baseArmSim.update(0.02); // standard loop time of 20ms
    simCurrentAngle = Units.radiansToDegrees(baseArmSim.getAngleRads()); //Returns angle in degrees
    armAngleSim.setDouble(simCurrentAngle);

  } 

  @Override
  public void close() throws Exception {
    // This method will close all device handles used by this object and release any other dynamic memory.
    // Mostly for JUnit tests
    motorBase.close(); 
  }
}
