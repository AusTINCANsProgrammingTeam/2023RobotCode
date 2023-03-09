// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.tofStates;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToFIntakeCommand extends CommandBase {
  /** Creates a new ToFIntakeCommand. */
  private final IntakeSubsystem intakeSubsystem;

  public ToFIntakeCommand(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tofStates tofState = intakeSubsystem.getFlightState();

    switch (tofState) {
      case IDLE:
        intakeSubsystem.stop();
        break;

      case CONE:
        intakeSubsystem.setMode(true);
        intakeSubsystem.pull();
        break;

      case CUBE:
        intakeSubsystem.setMode(false);
        intakeSubsystem.pull();
        break;

      case OFFLINE:
        // If one or more sensors are offline, do nothing
        break;
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
