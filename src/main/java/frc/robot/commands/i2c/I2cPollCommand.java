// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.i2c;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.VL53L0X;

/* Command to poll an I2C address on the Time of Flight sensor until
 * ANY of the bits in the bitmask are set (bitwise AND check).
 * 
 * Timeout is 5 seconds (250 cycles * 20 ms) by default, but can be overridden.
 */

public class I2cPollCommand extends CommandBase {
  private VL53L0X tof;
  private int index;
  private int result;
  private int bitmask;
  private int timeoutLoad, timeout;
  private final int timeoutDefault = 250;


  public I2cPollCommand(VL53L0X tof, int index, int bitmask) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tof);
    this.tof = tof;
    this.index = index;
    this.bitmask = bitmask;
    timeoutLoad = timeoutDefault;
    timeout = timeoutLoad;
  }
  public I2cPollCommand(VL53L0X tof, int index, int bitmask, int timeoutCycles) {
    this(tof, index, bitmask);
    timeoutLoad = timeoutCycles;
    timeout = timeoutLoad;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.isSimulation()) {
      // Finish polling on first read in simulation
      tof.setSimBuffer(bitmask, true); 
    }
    result = tof.readVL53L0X(index);
    timeout--;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timeout = timeoutLoad;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (result & bitmask) != 0 || timeout <= 0;
  }
}
