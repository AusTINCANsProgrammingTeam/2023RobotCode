
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {
  public static final double kConeIntakeSpeed = -0.75;
  public static final double kConeOuttakeSpeed = 0.75;
  public static final double kCubeIntakeSpeed = 0.55;
  public static final double kCubeOuttakeSpeed = -0.65;

  private CANSparkMax motor;
  private CANSparkMax motor2;
  private static ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
  private static GenericEntry intakeEntry = matchTab.add("Intake Speed", 0.0).getEntry();
  private static GenericEntry intakeMode = matchTab.add("Intake Mode", "Cone Mode").getEntry();

  private boolean isConeMode;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    isConeMode = true;

    motor = MotorController.constructMotor(MotorConfig.IntakeMotor1);
    motor2 = MotorController.constructMotor(MotorConfig.IntakeMotor2);
  }
  
  private void spinWheels(double velocity) {
    motor.set(velocity);
    motor2.set(-velocity);
    intakeEntry.setDouble(velocity);
  }

  public void push() {
    spinWheels(isConeMode ? kConeOuttakeSpeed : kCubeOuttakeSpeed);
  }

  public void pull() {
    spinWheels(isConeMode ? kConeIntakeSpeed : kCubeIntakeSpeed);
  }

    public void setConeMode() {
      this.isConeMode = true;
  }

    public void setCubeMode() {
      this.isConeMode = false;
  }

  public void toggleConeMode() {
    isConeMode = !isConeMode;
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
    SmartDashboard.putData(this);
    intakeMode.setString((isConeMode) ? "Cone Mode" : "Cube Mode");
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
