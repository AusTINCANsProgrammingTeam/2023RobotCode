// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

public class ArmCommand extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private final Supplier<Double> baseArmJoystick;
  private final Supplier<Double> elbowArmJoystick;
  private double elbowSetpoint = 0;
  private double baseSetpoint = 0;

  public ArmCommand(ArmSubsystem armSubsystem, Supplier<Double> baseArmJoystick, Supplier<Double> elbowArmJoystick) {
    this.armSubsystem = armSubsystem;
    this.baseArmJoystick = baseArmJoystick;
    this.elbowArmJoystick = elbowArmJoystick;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elbowSetpoint = armSubsystem.getElbowAngle();
    baseSetpoint = armSubsystem.getBaseAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!Robot.isSimulation()) {
      baseSetpoint = MathUtil.clamp(baseSetpoint+Units.degreesToRadians(baseArmJoystick.get()*2),Units.degreesToRadians(30),Units.degreesToRadians(100));
      armSubsystem.setBaseReference(baseSetpoint);
      elbowSetpoint = MathUtil.clamp(elbowSetpoint+Units.degreesToRadians(elbowArmJoystick.get()*2),Units.degreesToRadians(30),Units.degreesToRadians(130));
      armSubsystem.setElbowReference(elbowSetpoint);
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
