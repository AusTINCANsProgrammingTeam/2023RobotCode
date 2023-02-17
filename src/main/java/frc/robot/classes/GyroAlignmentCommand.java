// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.led.LedSubsystem;

public class GyroAlignmentCommand extends CommandBase {
  private LedSubsystem ledSubsystem_;
  private SwerveSubsystem swerveSubsystem;
  public GyroAlignmentCommand(LedSubsystem subsystem, SwerveSubsystem swervesubsystem) {
    ledSubsystem_ = subsystem;
    swerveSubsystem = swervesubsystem;
    addRequirements(subsystem);
    addRequirements(swervesubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
