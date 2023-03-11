// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.IntakeSubsystem.FlightStates;
import frc.robot.subsystems.IntakeSubsystem.FlightStates;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToFIntakeCommand extends CommandBase {
  /** Creates a new ToFIntakeCommand. */
  private final IntakeSubsystem intakeSubsystem;
  private final ArmSubsystem armSubsystem;
  private final double coneKeepSpeed = 0.35; // change after testing
  private final double cubeKeepSpeed = 0.25; // change after testing


  public ToFIntakeCommand(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.armSubsystem = armSubsystem;
    addRequirements(intakeSubsystem);
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    FlightStates tofState = intakeSubsystem.getFlightState();

    switch (tofState) {
      case IDLE:
        if (intakeSubsystem.getSpeed() != 0) {
          intakeSubsystem.stop();
        }
        break;

      case CONE:
        // If we're about to shoot, make sure not to stay running intake normally
        if (armSubsystem.getArmState().equals(ArmState.HIGHSCORE) || armSubsystem.getArmState().equals(ArmState.HIGHDROP)
        || armSubsystem.getArmState().equals(ArmState.MIDSCORE)) {
          tofState = FlightStates.CONE_SCORE;
        } else {
          intakeSubsystem.setMode(true);
          intakeSubsystem.spinWheels(coneKeepSpeed);
          break;
        }

      case CUBE:
        intakeSubsystem.setMode(false);
        intakeSubsystem.spinWheels(cubeKeepSpeed);
        break;

      case CONE_SCORE:
        // Turn off intake when about to score
        intakeSubsystem.stop();
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
