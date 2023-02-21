// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.i2c;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VL53L0X;

/* Command to read an I2C address on the Time of Flight sensor.
 *
 * Can retreive a single byte or word as an Integer, or a byte array for larger reads.
 * The returned value can be retrieved via the Consumer passed into the command constructor.
 * 
 */

public class I2cReadCommand extends CommandBase {
  private VL53L0X tof;
  private Consumer<Integer> consumer;
  private int index;
  private boolean byteNotWord;

  private Consumer<byte[]> listConsumer;
  private int count;

  /** Creates a new I2cRead8Command. */
  public I2cReadCommand(VL53L0X tof, Consumer<Integer> consumer, int index, boolean byteNotWord) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tof);
    this.tof = tof;
    this.index = index;
    this.consumer = consumer;
    this.byteNotWord = byteNotWord;
  }

  public I2cReadCommand(VL53L0X tof, Consumer<byte[]> consumer, int index, int count) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tof);
    this.tof = tof;
    this.index = index;
    this.listConsumer = consumer;
    this.count = count;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (consumer == null) {
      listConsumer.accept(tof.readBufferVL53L0X(index, count));
    } else {
      if (byteNotWord) {
        consumer.accept(tof.readVL53L0X(index));
      } else {
        consumer.accept(tof.read16VL53L0X(index));
      }
    }
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
