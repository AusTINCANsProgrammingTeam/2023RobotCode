// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;


public class ArmSubsystem extends SubsystemBase implements AutoCloseable {

  // FIXME using PWMSparkMax because CANSparkMax doesn't have an equivalent simulation class
  // May limit how much we can do in terms of JUnit tests
  private CANSparkMax motorBase;
  private CANSparkMax motorElbow;
  //private SparkMaxPIDController motorBase;
  private final RelativeEncoder motorBaseEncoder;
  private final RelativeEncoder motorElbowEncoder;
  private final SparkMaxPIDController armPIDController;
  private final SparkMaxPIDController elbowPIDController;

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    motorBase = MotorController.constructMotor(MotorConfig.ArmBaseMotor);
    motorElbow = MotorController.constructMotor(MotorConfig.ArmElbowMotor);
    //Encoders are not used right now, might be implemented later
    motorBaseEncoder = this.motorBase.getEncoder(); 
    motorElbowEncoder = this.motorElbow.getEncoder(); 
    armPIDController = motorBase.getPIDController();
    elbowPIDController = motorElbow.getPIDController();
  }

  @Override
  public void periodic() {  
    // This method will be called once per scheduler run
  }

  public void setBaseRef(double setpoint) {
    armPIDController.setReference(setpoint, ControlType.kPosition);
  }

   public void setElbowRef(double setpoint) {
    armPIDController.setReference(setpoint, ControlType.kPosition);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  } 

  @Override
  public void close() throws Exception {
    // This method will close all device handles used by this object and release any other dynamic memory.
    // Mostly for JUnit tests
    motorBase.close(); 
  }
}
