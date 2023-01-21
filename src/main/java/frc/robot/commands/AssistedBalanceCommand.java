// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SimulationSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AssistedBalanceCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem swerve_subsystem;
  private final SimulationSubsystem simulation_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AssistedBalanceCommand(SwerveSubsystem swerveSubsystem, SimulationSubsystem simulationSubsystem) {
    swerve_subsystem = swerveSubsystem;
    simulation_subsystem = simulationSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
    addRequirements(simulationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
                    
    //if (gyro.getPitch() != 0) {
                        
      if (simulation_subsystem.getPitch() < 360.0 && simulation_subsystem.getPitch() > 179.0) {
        SmartDashboard.putString("Direction", "Forward");
        swerve_subsystem.setModuleStates(swerve_subsystem.convertToModuleStates(0.0, 1.0, 0.0));
      }  
      else if (simulation_subsystem.getPitch() > 179 && simulation_subsystem.getPitch() < 181) {
        swerve_subsystem.setModuleStates(swerve_subsystem.convertToModuleStates(0.0, 0.0, 0.0));
      }
      else {
        swerve_subsystem.setModuleStates(swerve_subsystem.convertToModuleStates(0.0, -1.0, 0.0)); 
        SmartDashboard.putString("Direction", "Backward");
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
