// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import edu.wpi.first.wpilibj.CAN;

/** Add your docs here. */
public class TinCANBus extends CAN {

    /* 
      Extended Identifier bits:
        Device Type: 10 -> Miscellanious
        Manufacture ID: 8 -> Team Use
        API Class: 
           0 -> Device Config 
           API Indices:
             1 -> Set new deviceId

           1 -> I2C Bridge API
           API Indices:
             1 -> Set I2C device
             2 -> Send Read I2C 
             3 -> Send Write I2C
             4 -> Return last I2C read & status
             5 -> Periodic I2C Read

        Device Number/ID: 0-63
    */

    public TinCANBus(int deviceId) {
        super(deviceId);
    }


}

