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
             4 -> Get last I2C read
             5 -> Status
             6 -> Periodic I2C Read

        Device Number/ID: 0-63
    */

    private static int DEV_CONFIG_API_CLASS = 0;

    private static int SET_NEW_DEVICEID = 1;


    private static int I2C_BRIDGE_API_CLASS = 1;

    private static int   SET_DEVICE = 1;
    private static int   READ_I2C = 2;
    private static int   WRITE_I2C = 3;
    private static int   GET_READ = 4;
    private static int   STATUS = 5;
    private static int   PERIODIC_READ_I2C = 6;

    public TinCANBus(int deviceId) {
        super(deviceId);
    }

    private int getApiID(int apiClass, int apiIndex) {
        return ((apiClass & 0x003F) << 6) + (apiIndex & 0x0F);
    }

    public void setI2cDevice(byte i2cAddress) {
        byte [] packet = {i2cAddress};
        writePacket(packet, getApiID(I2C_BRIDGE_API_CLASS, SET_DEVICE));
    }

    public void requestI2cRead(byte regAddress) {

    }


}

