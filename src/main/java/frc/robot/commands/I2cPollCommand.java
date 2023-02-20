// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VL53L0X;

public class I2cPollCommand extends CommandBase {
  private VL53L0X tof;
  private int index;
  private int result;
  private int bitmask;
  private int timeout;


  /** Creates a new I2cRead8Command. */
  public I2cPollCommand(VL53L0X tof, int index, int bitmask) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tof);
    this.tof = tof;
    this.index = index;
    this.bitmask = bitmask;
    timeout = 250;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    result = tof.readVL53L0X(index);
    timeout--;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timeout = 250;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (result & bitmask) != 0 || timeout <= 0;
  }
}
