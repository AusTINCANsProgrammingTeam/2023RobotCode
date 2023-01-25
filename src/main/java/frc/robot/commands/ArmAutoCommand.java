// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.lang.Math;

public class ArmAutoCommand extends CommandBase {
  //TODO: Move these to a constants file, all measurements are in cm
  private static final double firstArmLength = 95.25;
  private static final double secondArmLength = 100;

  private final ArmSubsystem armSubsystem;
  private double firstArmAngle;
  private double secondArmAngle;
  private double currentX;
  private double currentY;

  public ArmAutoCommand(ArmSubsystem armSubsystem, double x, double y) {
    this.armSubsystem = armSubsystem;
    getAngles(x, y);
    currentX = x;
    currentY= y;
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
  public static double getPrelimAngle(double x, double y) {
    return (
      -1*Math.acos(
        ( (x*x) + (y*y) - (firstArmLength*firstArmLength) - (secondArmLength*secondArmLength) )/(2*firstArmLength*secondArmLength)
      )
    );
  }
  public static double getBaseAngle(double x, double y) {
    double pAngle = (
      -1*Math.acos(
        ( (x*x) + (y*y) - (firstArmLength*firstArmLength) - (secondArmLength*secondArmLength) )/(2*firstArmLength*secondArmLength)
      )
    );
    double fArmAngle = (
            Math.atan(y/x)-Math.atan((secondArmLength*Math.sin(pAngle) )/( (firstArmLength+(secondArmLength*Math.cos(pAngle))))
    ));
    return fArmAngle;
  }
  
  public static double getElbowAngle(double x, double y) {
    double pAngle = (
      -1*Math.acos(
        ( (x*x) + (y*y) - (firstArmLength*firstArmLength) - (secondArmLength*secondArmLength) )/(2*firstArmLength*secondArmLength)
      )
    );
    double fArmAngle = (
            Math.atan(y/x)-Math.atan((secondArmLength*Math.sin(pAngle) )/( (firstArmLength+(secondArmLength*Math.cos(pAngle))))
    ));
    return fArmAngle+pAngle;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Needs to move to points x,y
    getAngles(currentX, currentY);
    armSubsystem.setBaseRef(firstArmAngle);
    armSubsystem.setElbowRef(secondArmAngle);
  }
  public void setBaseArmAngle(double angle) {
    armSubsystem.setBaseRef(angle);
  }
  public void setBaseArmPos(double x, double y) {
    armSubsystem.setBaseRef(Math.atan(y/x));
  }
  public void changeBaseArmX(double xchange) {
    currentX = Math.cos(firstArmAngle) * firstArmLength;
    currentY = Math.sin(firstArmAngle) * firstArmLength;
    setBaseArmPos(currentY,(currentX+xchange));
  }
  public void changeBaseArmY(double ychange) {
    currentX = Math.cos(firstArmAngle) * firstArmLength;
    currentY = Math.sin(firstArmAngle) * firstArmLength;
    setBaseArmPos((currentY+ychange),currentX);
  }
  public void changeBaseArmCoords(double xchange, double ychange) {
    changeBaseArmX(xchange);
    changeBaseArmY(ychange);
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
