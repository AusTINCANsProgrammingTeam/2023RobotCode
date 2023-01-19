
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {
  public static final double kOuttakeSpeed = -1;
  public static final double kIntakeSpeed = 0.5;
  private CANSparkMax motor;
  private CANSparkMax motor2;
  private ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake Status");
  private GenericEntry intakeEntry = intakeTab.add("Intake Speed", 0.0).getEntry();
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    motor = MotorController.constructMotor(MotorConfig.IntakeMotor1);
    motor2 = MotorController.constructMotor(MotorConfig.IntakeMotor2);
    motor.follow(motor2);
  }
  
  private void spinWheels(double velocity) {
    motor.set(velocity);

  }

  public void push() {
    spinWheels(kOuttakeSpeed);
  }

  public void pull() {
    spinWheels(kOuttakeSpeed);
  }
  
  public double getSpeed(){
    return motor.get();
  }

  @Override
  public void close() throws Exception {
    // This method will close all device handles used by this object and release any other dynamic memory.
    // Mostly for JUnit tests
    motor.close();
    motor2.close();
    
  }

  @Override
  public void periodic() {
         double RPS = getSpeed();
     intakeEntry.setDouble(RPS);
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
