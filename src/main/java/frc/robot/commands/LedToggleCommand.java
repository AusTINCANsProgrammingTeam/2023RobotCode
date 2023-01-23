// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LedSubsystem;

public class LedToggleCommand extends CommandBase {
  //Controller Port 1 Button 1

  private final LedSubsystem m_ledSubsystem;
  public static boolean LedToggle = false;
  /** Creates a new LedCommand. */
  public LedToggleCommand(LedSubsystem ledSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ledSubsystem = ledSubsystem;
    addRequirements(m_ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LedSubsystem.leds.start();
    LedToggle = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LedSubsystem.leds.stop();
    LedToggle = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
