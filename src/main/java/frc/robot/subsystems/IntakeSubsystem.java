// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {

  private CANSparkMax motor;
  private CANSparkMax motor2;
  private static final double OUTTAKE_SPEED = -1;
  private static final double INTAKE_SPEED = 0.5;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    motor = new CANSparkMax(9, MotorType.kBrushless);
    motor2 = new CANSparkMax(10, MotorType.kBrushless);
    motor.follow(motor2);
    motor.setInverted(true);
  }

  private void spinWheels(double velocity) {
    motor.set(velocity);

  }

  public void push() {
    spinWheels(OUTTAKE_SPEED);
  }

  public void pull() {
    spinWheels(INTAKE_SPEED);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void close() throws Exception {
    // This method will close all device handles used by this object and release any other dynamic memory.
    // Mostly for JUnit tests
    motor.close();
    
  }
}
