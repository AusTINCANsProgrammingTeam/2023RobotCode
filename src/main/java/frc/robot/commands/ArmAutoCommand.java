// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.lang.Math;

public class ArmAutoCommand extends CommandBase {
  //TODO: Move these to a constants file, all measurements are in cm
  private static final double firstArmLength = 76.2;
  private static final double secondArmLength = 144.3;

  private final ArmSubsystem armSubsystem;
  private double firstArmAngle;
  private double secondArmAngle;

  public ArmAutoCommand(ArmSubsystem armSubsystem, double x, double y) {
    this.armSubsystem = armSubsystem;
    getAngles(x, y);
    addRequirements(armSubsystem);
  }
  private void getAngles(double x, double y) {
    //Needed to calculate the first and second arm angles
    double prelimAngle = (
      -1*Math.acos(
        ( (x*x) + (y*y) - (firstArmLength*firstArmLength) - (secondArmLength*secondArmLength) )/(2*firstArmLength*secondArmLength)
      )
    );
    //Get angle of the first arm (relative to a flat bottom line) (in radians)
    firstArmAngle = (
            Math.atan(y/x)-Math.atan((secondArmLength*Math.sin(prelimAngle) )/( (firstArmLength+(secondArmLength*Math.cos(prelimAngle))))
    ));
    //Get angle of the second arm relative to the first one (in radians)
    secondArmAngle = prelimAngle+firstArmAngle;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Needs to move to points x,y
    armSubsystem.setBaseRef(firstArmAngle);
    armSubsystem.setElbowRef(secondArmAngle);
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
