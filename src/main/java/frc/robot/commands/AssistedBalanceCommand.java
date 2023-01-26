// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.classes.TunableNumber;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An AssistedBalaceCommand command that uses SwerveSubsystem and SimulationSubsystemd */
public class AssistedBalanceCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem swerve_subsystem;
  private final double kPBalancing = 0.2;
  private final double kIBalancing = 0;
  private final double kDBalancing = 0;
  private final double balancingDeadzoneNumber = 0.001;
  private PIDController pidController = new PIDController(kPBalancing, kIBalancing, kDBalancing);
  TunableNumber tunableP = new TunableNumber("Balancing P", kPBalancing, pidController::setP);
  TunableNumber tunableI = new TunableNumber("Balancing I", kIBalancing, pidController::setI);
  TunableNumber tunableD = new TunableNumber("Balancing D", kDBalancing, pidController::setD);

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

    swerve_subsystem.setModuleStates(swerve_subsystem.convertToModuleStates(0.0, pidController.calculate(swerve_subsystem.getPitch(), 0.0), 0.0));  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve_subsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerve_subsystem.getPitch() < balancingDeadzoneNumber && swerve_subsystem.getPitch() > -balancingDeadzoneNumber;
  }
}
