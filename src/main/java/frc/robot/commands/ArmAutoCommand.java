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
  private double ytheta;
  private double xtheta;

  public ArmAutoCommand(ArmSubsystem armSubsystem, double x, double y) {
    this.armSubsystem = armSubsystem;
    getAngles(x, y);
    addRequirements(armSubsystem);
  }

  private double findHypotenuse(double x, double y) {
    return Math.sqrt((x*x)+(y*y));
  }
  private void getAngles(double x, double y) {
    //Get angle of the second arm relative to the first one (in radians)
    ytheta = (
      Math.acos(
        ( (x*x) + (y*y) - (firstArmLength*firstArmLength) - (secondArmLength*secondArmLength) )/(2*firstArmLength*secondArmLength)
      )
    );
    //Get angle of the first arm (relative to a flat bottom line) (in radians)
    xtheta = (
            Math.atan(y/x)-Math.atan(secondArmLength*Math.sin(104.56) / (firstArmLength+(secondArmLength*Math.cos(104.56)))
    ));
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Needs to move to points x,y
    armSubsystem.setBaseRef(xtheta);
    armSubsystem.setElbowRef(ytheta);
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
