// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.classes.DebugLog;

public class CubeapultSubsystem extends SubsystemBase {
  private static final double kStartPos = 0;
  private static final double kStoredPos = 0.5;
  private static final double kLaunchPos = 1;
  private static final int servoID = 0;
  private Servo servo;

  private DebugLog<Double> actualServoPositionLog = new DebugLog<Double>(0.0, "Actual Servo Position", this);
  private DebugLog<Double> desiredServoPositionLog = new DebugLog<Double>(0.0, "Desired Servo Position", this);
  
  public CubeapultSubsystem() {
    servo = new Servo(servoID);
    start();
  }

  private void start() {
    servo.set(kStartPos);
    desiredServoPositionLog.log(kStartPos);
  }

  private void store() {
    servo.set(kStoredPos);
    desiredServoPositionLog.log(kStoredPos);
  }

  private void launch() {
    servo.set(kLaunchPos);
    desiredServoPositionLog.log(kLaunchPos);
  }

  public Command launchCube() {
    return new SequentialCommandGroup(
      new InstantCommand(this::launch),
      new WaitCommand(0.5),
      new InstantCommand(this::store)
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    actualServoPositionLog.log(servo.get());
  }
}