
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.TunableNumber;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {
  public static final double kOuttakeSpeed = 0.2;
  public static final double kIntakeSpeed = -0.75;
  private CANSparkMax motor;
  private CANSparkMax motor2;
  private static ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
  private static GenericEntry intakeEntry = matchTab.add("Intake Speed", 0.0).getEntry();

  private TunableNumber speedOneTuner = new TunableNumber("Motor 4", 0, (a) -> {});
  private TunableNumber speedTwoTuner = new TunableNumber("Motor 5", 0, (a) -> {});

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    motor = MotorController.constructMotor(MotorConfig.IntakeMotor1);
    motor2 = MotorController.constructMotor(MotorConfig.IntakeMotor2);
  }
  
  private void spinWheels(double velocity) {
    motor.set(velocity);
    motor2.set(-velocity);
    intakeEntry.setDouble(velocity);
  }

  public void push() {
    motor.set(0.2);
    motor2.set(-0.2);
    motor.setIdleMode(IdleMode.kCoast);
    motor2.setIdleMode(IdleMode.kCoast);
  }

  public void pull() {
    spinWheels(kIntakeSpeed);
    motor.setIdleMode(IdleMode.kBrake);
    motor2.setIdleMode(IdleMode.kBrake);
  }
  
  public double getSpeed(){
    return motor.get();
  }

  public void stop() {
    spinWheels(0);
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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
