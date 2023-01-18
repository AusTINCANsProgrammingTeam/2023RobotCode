// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;


public class ArmSubsystem extends SubsystemBase implements AutoCloseable {

  // FIXME using PWMSparkMax because CANSparkMax doesn't have an equivalent simulation class
  // May limit how much we can do in terms of JUnit tests
  private double P = 0.001;
  private double I = 0.001;
  private double D = 0.001;
  private CANSparkMax motorBase;
  private CANSparkMax motorElbow;
  private final RelativeEncoder motorBaseEncoder;
  private final RelativeEncoder motorElbowEncoder;
  private final PIDController basePIDController;
  private final PIDController elbowPIDController;

  // SIM VALUES:
  private double gearing = 1;
  private double baseArmLength = Units.inchesToMeters(40);
  private double minAngle = Units.degreesToRadians(-160);
  private double maxAngle = Units.degreesToRadians(160);
  private double baseArmMass = Units.lbsToKilograms(15);
  private final double baseArmInertia = SingleJointedArmSim.estimateMOI(baseArmLength, baseArmMass);

  private final SingleJointedArmSim singleJointedArmSim = new SingleJointedArmSim(DCMotor.getNEO(1), gearing, baseArmInertia, 
  baseArmLength, minAngle, maxAngle, baseArmMass, false);

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
  } 

  @Override
  public void close() throws Exception {
    // This method will close all device handles used by this object and release any other dynamic memory.
    // Mostly for JUnit tests
    motorBase.close(); 
  }
}
