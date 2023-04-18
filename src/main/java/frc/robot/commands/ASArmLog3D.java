// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ASArmLog3D extends CommandBase {
  /** Creates a new ASArmLog3D. */
  private ArmSubsystem armSubsystem;
  private SwerveSubsystem swerveSubsystem;

  public ASArmLog3D(ArmSubsystem armSubsystem, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
    addRequirements(swerveSubsystem);
    this.armSubsystem = armSubsystem;
    this.swerveSubsystem = swerveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d swervePose = swerveSubsystem.getPose();
    Logger.getInstance().recordOutput("Base Arm Pose3D",  new Pose3d(0.001289, -0.252741, -0.247927,new Rotation3d(-armSubsystem.getBaseAngle(), 0, 0)));
    Logger.getInstance().recordOutput("Elbow Arm Pose3D", new Pose3d(-0.009661, armSubsystem.getArmY()-0.255489, -0.793469, new Rotation3d(armSubsystem.getElbowAngle(), 0, 0)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
