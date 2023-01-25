// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An AssistedBalaceCommand command that uses SwerveSubsystem and SimulationSubsystemd */
public class AssistedBalanceCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem swerve_subsystem;
  PIDController pidController = new PIDController(0.2, 0, 0);

  /**
   * Creates a new AssistedBalanceCommand
   *
   * @param subsystem The subsystem used by this command.
   */
  public AssistedBalanceCommand(SwerveSubsystem swerveSubsystem) {
    swerve_subsystem = swerveSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    swerve_subsystem.setModuleStates(swerve_subsystem.convertToModuleStates(pidController.calculate(swerve_subsystem.getPitch(), 0.0), 0.0, 0.0));

     /* if (swerve_subsystem.getPitch() > Units.degreesToRadians(2) && swerve_subsystem.getPitch() < 180.0) {
        SmartDashboard.putString("Direction", "Forward");
        swerve_subsystem.setModuleStates(swerve_subsystem.convertToModuleStates(-1.0, 0.0, 0.0));
      }  
      else if (swerve_subsystem.getPitch() > Units.degreesToRadians(-2) && swerve_subsystem.getPitch() < Units.degreesToRadians(2)) {
        swerve_subsystem.stopModules();
      }
      else if (swerve_subsystem.getPitch() > -180.0 && swerve_subsystem.getPitch() < Units.degreesToRadians(-2)) {
        swerve_subsystem.setModuleStates(swerve_subsystem.convertToModuleStates(1.0, 0.0, 0.0)); 
        SmartDashboard.putString("Direction", "Backward");
    }
    else {
      swerve_subsystem.setModuleStates(swerve_subsystem.convertToModuleStates(0.0, 0.0, 0.0));
    }*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //TODO: Lock when wheels are balanced
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
