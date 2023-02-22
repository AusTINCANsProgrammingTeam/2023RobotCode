
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Robot;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {
  public static final double kOuttakeSpeed = -1;
  public static final double kIntakeSpeed = 0.5;
  public static final double kRetractPosition = -.5; //TODO measure and find position 
  public static final double kExtendPosition = .5;
  private CANSparkMax motor;
  private CANSparkMax motor2;
  private CANSparkMax motor3;
  private boolean inOutArm = false;
  private static ShuffleboardTab matchTab;
  public static ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");
  private static GenericEntry intakeEntry = competitionTab.add("Intake Speed", "").getEntry();

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    motor = MotorController.constructMotor(MotorConfig.IntakeMotor1);
    motor2 = MotorController.constructMotor(MotorConfig.IntakeMotor2);
    motor.follow(motor2);

    if (!Robot.isCompetition){
      matchTab = Shuffleboard.getTab("Match");
      intakeEntry = matchTab.add("Intake Speed", 0.0).getEntry();
    }
  }
  
  private void spinWheels(double velocity) {
    motor.set(velocity);
    if (!Robot.isCompetition) {
      intakeEntry.setDouble(velocity);
    }
  }

  public void push() {
    spinWheels(kOuttakeSpeed);
  }

  public void pull() {
    spinWheels(kIntakeSpeed);
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

public void inOutArm() {
  inOutArm = !inOutArm; //Toggle boolean
    if (inOutArm) {
    motor3.getPIDController().setReference(kExtendPosition,ControlType.kPosition);
  } else {
    motor3.getPIDController().setReference(kRetractPosition,ControlType.kPosition);
  }
}
}
