// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

/** An AssistedBalaceCommand command that uses SwerveSubsystem and SimulationSubsystemd */
public class AssistedBalanceCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem swerve_subsystem;
  private final double kPBalancing = 0.008;
  private final double kIBalancing = 0;
  private final double kDBalancing = 0;
  private final double balancingDeadzoneNumber = 2.5;
  private double pidControllerMaxSpeed = 0.15;
  private boolean reversed = false;
  PIDController pidController = new PIDController(kPBalancing, kIBalancing, kDBalancing);
  
  /**
   * Creates a new AssistedBalanceCommand
   *
   * @param subsystem The subsystem used by this command.
   */
  public AssistedBalanceCommand(SwerveSubsystem swerveSubsystem, boolean reverse) {
    swerve_subsystem = swerveSubsystem;
    pidController.setTolerance(balancingDeadzoneNumber);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem); 
    reversed = reverse;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve_subsystem.setModuleStates(
      swerve_subsystem.convertToModuleStates(
        0.0, MathUtil.clamp((reversed?-1:1)*pidController.calculate(swerve_subsystem.getPitch(), 0.0), -pidControllerMaxSpeed, pidControllerMaxSpeed), 0.0));  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve_subsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
