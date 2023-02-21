// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

public class ArmAnglesCommand extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private final Supplier<Double> baseArmJoystick;
  private final Supplier<Double> elbowArmJoystick;
  private final ArmState state;

  public ArmAnglesCommand(ArmSubsystem armSubsystem, Supplier<Double> baseArmJoystick, Supplier<Double> elbowArmJoystick, ArmState state) {
    this.armSubsystem = armSubsystem;
    this.baseArmJoystick = baseArmJoystick;
    this.elbowArmJoystick = elbowArmJoystick;
    this.state = state;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.setState(state);
    //armSubsystem.setReferences(armSubsystem.getBaseAngle(), armSubsystem.getElbowAngle());
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
