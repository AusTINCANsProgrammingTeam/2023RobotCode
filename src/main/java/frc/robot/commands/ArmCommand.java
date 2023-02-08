// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

public class ArmCommand extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private final Supplier<Double> rJoystick;
  private double setpoint = 0;

  public ArmCommand(ArmSubsystem armSubsystem, Supplier<Double> rJoystick) {
    this.armSubsystem = armSubsystem;
    this.rJoystick = rJoystick;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpoint = armSubsystem.getElbowAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //armSubsystem.stopArmMotors();
    if(ArmSubsystem.controlIsBaseArm) {
      //setpoint = MathUtil.clamp(setpoint+Units.degreesToRadians(rJoystick.get()*2),Units.degreesToRadians(30),Units.degreesToRadians(100));
      //armSubsystem.setBaseReference(setpoint);
      //SmartDashboard.putNumber("Base setpoint", Units.radiansToDegrees(setpoint));
      armSubsystem.setBase(rJoystick.get());
      SmartDashboard.putNumber("Controller", rJoystick.get());
      //Move base arm
      //armSubsystem.setBaseRef(rJoystick.get());
      //armSubsystem.setBaseRef((MathUtil.clamp(rJoystick.get(),0,1)*Units.degreesToRadians(45))+Units.degreesToRadians(45));
    } else {
      
      setpoint = MathUtil.clamp(setpoint+Units.degreesToRadians(rJoystick.get()*2),Units.degreesToRadians(30),Units.degreesToRadians(130));
      armSubsystem.setElbowReference(setpoint);
      SmartDashboard.putNumber("Elbow setpoint", Units.radiansToDegrees(setpoint));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
