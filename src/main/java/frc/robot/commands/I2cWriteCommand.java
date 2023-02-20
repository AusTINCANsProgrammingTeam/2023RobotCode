// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.Pair;

import frc.robot.subsystems.VL53L0X;

public class I2cWriteCommand extends CommandBase {
  private VL53L0X tof;
  private List<Pair<Integer,Integer>> pairs;
  private int listIndex = 0;
  private boolean byteNotWord;

  private int index;
  private byte[] buffer;


  /** Creates a new I2cRead8Command. */
  public I2cWriteCommand(VL53L0X tof, int index, int data, boolean byteNotword) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tof);
    this.tof = tof;
    pairs = List.of(new Pair<Integer,Integer>(index, data));

  }

  public I2cWriteCommand(VL53L0X tof, List<Pair<Integer,Integer>> pairs, boolean byteNotWord) {

    addRequirements(tof);
    this.tof = tof;
    this.pairs = pairs;
    this.byteNotWord = byteNotWord;
    
  }

  public I2cWriteCommand(VL53L0X tof, int index, byte[] buf) {

    addRequirements(tof);
    this.tof = tof;
    buffer = buf;
    this.index = index;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (pairs == null) {
      tof.writeBufferVL53L0X(index, buffer);

    } else {
      if (byteNotWord) {
        tof.writeVL53L0X(pairs.get(listIndex).getFirst(), pairs.get(listIndex).getSecond());
      } else {
        tof.write16VL53L0X(pairs.get(listIndex).getFirst(), pairs.get(listIndex).getSecond());
      }
    }
    listIndex++;


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pairs == null || listIndex >= pairs.size();
  }
}
