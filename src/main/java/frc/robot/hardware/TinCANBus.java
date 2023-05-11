// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;

/*
 * Implements an API for a custom CAN device
 * 
 * See FRC CAN Bus Documentation: https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html
 */
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

        Device Number/ID: 0-63
    */


    private static enum DEV_CONFIG_API_CLASS {
      SET_NEW_DEVICEID(1);

      private final int APICLASS = 0;
      private final int APIID;
      
      DEV_CONFIG_API_CLASS(int apiindex) {
        this.APIID = getApiID(APICLASS, apiindex);
      }

    }

    private static enum SENSOR_BRIDGE_API_CLASS {
      GET_READ(1),
      STATUS(2);

      private final int APICLASS = 1;
      private final int APIID;
      
      SENSOR_BRIDGE_API_CLASS(int apiindex) {
        this.APIID = getApiID(APICLASS, apiindex);
      }
    }

    // API ID is a 10 bit sequence 
    // Bits 9-4 -> API Class 
    // Bits 3-0 -> API Index 
    private static int getApiID(int apiClass, int apiIndex) {
        return ((apiClass & 0x003F) << 4) + (apiIndex & 0x0F);
    }

    private static int byteArrayToInt(byte[] array, int length) {
      int ret = 0;
      for (int i = 0; i < array.length && i < length && i < Integer.BYTES; i++) {
        ret = (ret << 8) + (array[i] & 0x00FF);
      }
      return ret;
    }


    public TinCANBus(int deviceId) {
        super(deviceId);
    }

    // Send packet to tell CAN device to change the device number it listens for. 
    // Will be applied when the CAN device is next power cycled
    // Only use when only the CAN device of this type you want to change is connected to the bus
    public void setNewDeviceId(int deviceId) {
      byte[] buff = {(byte)deviceId};
      writePacket(buff, DEV_CONFIG_API_CLASS.SET_NEW_DEVICEID.APIID);
    }

    // Returns 2 bytes representing the last sensor read value 
    // or -1 if packet error occurs
    public int lastSensorRead() {
        CANData d = new CANData();
        if (readPacketLatest(SENSOR_BRIDGE_API_CLASS.GET_READ.APIID, d)) {
          return byteArrayToInt(d.data, 2);
        } else {
          return -1;
        }
    }

    // Return 1 byte status message (TBD)
    // or -1 if packet error occurs
    public int getSensorStatus() {
        CANData d = new CANData();
        if (readPacketLatest(SENSOR_BRIDGE_API_CLASS.STATUS.APIID, d)) {
          return byteArrayToInt(d.data, 1);
        } else {
          return -1;
        }
    }


}

