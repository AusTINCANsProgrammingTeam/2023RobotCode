// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import edu.wpi.first.hal.CANData;
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

           1 -> Sensor Bridge API
           API Indices:
             1 -> Get last sensor read
             2 -> Status
             3 -> Periodic sensor Read

        Device Number/ID: 0-63
    */

    private static int DEV_CONFIG_API_CLASS = 0;

    private static int SET_NEW_DEVICEID = 1;


    private static int SENSOR_BRIDGE_API_CLASS = 1;

    private static int   GET_READ = 1;
    private static int   STATUS = 2;
    private static int   PERIODIC_READ = 3;

    public TinCANBus(int deviceId) {
        super(deviceId);
    }

    private int getApiID(int apiClass, int apiIndex) {
        return ((apiClass & 0x003F) << 4) + (apiIndex & 0x0F);
    }


    public int lastSensorRead() {
        CANData d = new CANData();
        readPacketLatest(getApiID(SENSOR_BRIDGE_API_CLASS, GET_READ), d);
        return ((int)d.data[0]) & 0x00FF + ((((int)d.data[1]) & 0x00FF) << 8);



    }


}

