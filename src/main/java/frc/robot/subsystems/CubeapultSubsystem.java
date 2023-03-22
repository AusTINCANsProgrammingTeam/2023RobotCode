// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.classes.DebugLog;

public class CubeapultSubsystem extends SubsystemBase {
  private static final int solenoidForwardID = 1; //TODO: Update ids when plugged in to the robot
  private static final int solenoidReverseID = 0; //TODO: Update ids when plugged in to the robot
  private static final int pneumaticsID = 59;
  private static final int activationMaxPressure = 120;
  private static final int activationMinPressure = 100;
  private DoubleSolenoid solenoid;
  private PneumaticHub pneumatics;
  private ShuffleboardTab configTab = Shuffleboard.getTab("Config");
  private DebugLog<Boolean> solenoidStateLog = new DebugLog<Boolean>(false, "Solenoid State", this);
  
  public CubeapultSubsystem() {
    configTab.add(new StartEndCommand(this::activate, this::deactivate, this).withName("Enable Compressor"));
    solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, solenoidForwardID, solenoidReverseID);
    pneumatics = new PneumaticHub();
    retract();
    deactivate();
  }

  private void extend() {
    solenoid.set(Value.kForward);;
    solenoidStateLog.log(false);
  }

  private void retract() {
    solenoid.set(Value.kReverse);
    solenoidStateLog.log(true);
  }

  private void activate() {
    pneumatics.enableCompressorAnalog(activationMinPressure, activationMaxPressure);
  }

  private void deactivate() {
    pneumatics.disableCompressor();
  }

  public Command launch() {
    return new SequentialCommandGroup(
      new InstantCommand(this::extend, this),
      new WaitCommand(0.5),
      new InstantCommand(this::retract, this)
    ).withName("Launch");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData(this);
  }
}