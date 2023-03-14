// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

public class ArmAnglesCommand extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private final Supplier<Double> baseArmJoystick;
  private final Supplier<Double> elbowArmJoystick;

  public ArmAnglesCommand(ArmSubsystem armSubsystem, Supplier<Double> baseArmJoystick, Supplier<Double> elbowArmJoystick) {
    this.armSubsystem = armSubsystem;
    this.baseArmJoystick = baseArmJoystick;
    this.elbowArmJoystick = elbowArmJoystick;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.updateReferences(baseArmJoystick.get(), elbowArmJoystick.get());
    armSubsystem.updateMotors();
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
