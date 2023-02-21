// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.i2c;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.Pair;

import frc.robot.subsystems.VL53L0X;

/* Command to write to an I2C address on the Time of Flight sensor.
 * 
 * Can write single bytes or words, address:data pairs from a List<Pair<Integer,Integer>> constant,
 * or byte array data for multi-byte writes.
 * 
 * Supplier versions of the constructors are availible to allow write data to change after the 
 * I2cWriteCommand has been created.
 */

public class I2cWriteCommand extends CommandBase {
  private VL53L0X tof;
  private List<Pair<Integer,Integer>> pairs;
  private int listIndex = 0;
  private boolean byteNotWord;

  private int index;
  private Supplier<byte[]> bufferSupplier;


  // Use this constructor if all inputs are constant values
  public I2cWriteCommand(VL53L0X tof, int index, int data, boolean byteNotword) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tof);
    this.tof = tof;
    pairs = List.of(new Pair<Integer,Integer>(index, data));

  }

  /* Use this constructor if the data value is a variable;
   * i.e. if the value of the data may change between the time 
   * this Command is constructed and when it is scheduled
   * 
   * e.g. I2cWriteCommand(tof, 0x83, () -> {return dataVariable;}, true)
   */
  public I2cWriteCommand(VL53L0X tof, int index, Supplier<Integer> data, boolean byteNotword) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tof);
    this.tof = tof;
    if (byteNotword) {
      bufferSupplier = () -> {
        byte[] buf = new byte[1];
        buf[0] = (byte) (data.get() & 0xFF);
        return buf;
      };
  } else {
      bufferSupplier = () -> {
        byte[] buf = new byte[2];
        buf[0] = (byte) ((data.get() & 0xFF00) >> 8);
        buf[1] = (byte) (data.get() & 0xFF);
        return buf;
      };

  }

  }

  /* Use this constructor to write a (constant) list of address:data pairs.
   * Note that there is no Supplier version for this I2cWriteCommand type
   */
  public I2cWriteCommand(VL53L0X tof, List<Pair<Integer,Integer>> pairs, boolean byteNotWord) {

    addRequirements(tof);
    this.tof = tof;
    this.pairs = pairs;
    this.byteNotWord = byteNotWord;
    
  }

  /* Constructs an I2cWriteCommand using a data buffer.
   * Use if multi-byte data needs to be written from a constant.
   */
  public I2cWriteCommand(VL53L0X tof, int index, byte[] buf) {

    addRequirements(tof);
    this.tof = tof;
    bufferSupplier = () -> {return buf;};
    this.index = index;
  }

  /* Constructs an I2cWriteCommand using a data buffer created by a Supplier method
   * Use if multi-byte data needs to be written from a variable;
   * i.e. if the value of the data may change between the time 
   * this Command is constructed and when it is scheduled
   * 
   * e.g. I2cWriteCommand(tof, 0x83, () -> {return bufferVariable;}, true)
   */
  public I2cWriteCommand(VL53L0X tof, int index, Supplier<byte[]> buf) {

    addRequirements(tof);
    this.tof = tof;
    bufferSupplier = buf;
    this.index = index;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (pairs == null) {
      tof.writeBufferVL53L0X(index, bufferSupplier.get());

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
  public void end(boolean interrupted) {
    listIndex = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pairs == null || listIndex >= pairs.size();
  }
}
