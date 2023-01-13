// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;

public class ArmCommand extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private final Joystick baseJoystick;
  private final int minSetpoint = 5;
  private final int maxSetpoint = 30;

  public ArmCommand(ArmSubsystem armSubsystem, Joystick baseJoystick) {
    this.armSubsystem = armSubsystem;
    this.baseJoystick = baseJoystick;

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double baseY = baseJoystick.getY();
    double baseSetpoint;

    if (baseY >= 0) {
      baseSetpoint = baseY * maxSetpoint;
    } else {
      baseSetpoint = baseY * minSetpoint;
    }

    armSubsystem.setBaseRef(baseSetpoint);
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
