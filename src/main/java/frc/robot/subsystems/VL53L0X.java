// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.simulation.BufferCallback;
import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.simulation.CallbackStore;
import edu.wpi.first.wpilibj.simulation.I2CSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import java.util.List;

/** Add your docs here. */
public class VL53L0X  extends SubsystemBase implements AutoCloseable{
    private final boolean DEBUG = false;
    private static final int i2c_addr = 0x29;
    private final I2C i2c; 


    public static final int SYSRANGE_START = 0x00;
    public static final int SYSTEM_THRESH_HIGH = 0x0C;
    public static final int SYSTEM_THRESH_LOW = 0x0E;
    public static final int SYSTEM_SEQUENCE_CONFIG = 0x01;
    public static final int SYSTEM_RANGE_CONFIG = 0x09;
    public static final int SYSTEM_INTERMEASUREMENT_PERIOD = 0x04;
    public static final int SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A;
    public static final int GPIO_HV_MUX_ACTIVE_HIGH = 0x84;
    public static final int SYSTEM_INTERRUPT_CLEAR = 0x0B;
    public static final int RESULT_INTERRUPT_STATUS = 0x13;
    public static final int RESULT_RANGE_STATUS = 0x14;
    public static final int RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC;
    public static final int RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = 0xC0;
    public static final int RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0;
    public static final int RESULT_CORE_RANGING_TOTAL_EVENTS_REF = 0xD4;
    public static final int RESULT_PEAK_SIGNAL_RATE_REF = 0xB6;
    public static final int ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28;
    public static final int I2C_SLAVE_DEVICE_ADDRESS = 0x8A;
    public static final int MSRC_CONFIG_CONTROL = 0x60;
    public static final int PRE_RANGE_CONFIG_MIN_SNR = 0x27;
    public static final int PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56;
    public static final int PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57;
    public static final int PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64;
    public static final int FINAL_RANGE_CONFIG_MIN_SNR = 0x67;
    public static final int FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47;
    public static final int FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48;
    public static final int FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44;
    public static final int PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61;
    public static final int PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62;
    public static final int PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50;
    public static final int PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51;
    public static final int PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52;
    public static final int SYSTEM_HISTOGRAM_BIN = 0x81;
    public static final int HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33;
    public static final int HISTOGRAM_CONFIG_READOUT_CTRL = 0x55;
    public static final int FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70;
    public static final int FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71;
    public static final int FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72;
    public static final int CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20;
    public static final int MSRC_CONFIG_TIMEOUT_MACROP = 0x46;
    public static final int SOFT_RESET_GO2_SOFT_RESET_N = 0xBF;
    public static final int IDENTIFICATION_MODEL_ID = 0xC0;
    public static final int IDENTIFICATION_REVISION_ID = 0xC2;
    public static final int OSC_CALIBRATE_VAL = 0xF8;
    public static final int GLOBAL_CONFIG_VCSEL_WIDTH = 0x32;
    public static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0;
    public static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0xB1;
    public static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0xB2;
    public static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0xB3;
    public static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0xB4;
    public static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0xB5;
    public static final int GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6;
    public static final int DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E;
    public static final int DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F;
    public static final int POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80;
    public static final int VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89;
    public static final int ALGO_PHASECAL_LIM = 0x30;
    public static final int ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30;
    public static final int VCSEL_PERIOD_PRE_RANGE = 0;
    public static final int VCSEL_PERIOD_FINAL_RANGE = 1;

    // If an I2C transaction fails, this gets set to false and future transactions are ignored. 
    // TODO make this better
    private boolean isPresent = true;
    private int range_mm = 0;

    // Simulation variables
    private I2CSim simDev;
    private CallbackStore readCallbackStore, writeCallbackStore;
    private final int simBufSize = 128; // Buffer is bigger than it needs to be to avoid index out of range exceptions.
    private byte[] simBuf = new byte[simBufSize]; 

/* Vendor of this IC doesn't provide a register map (apparently its extremely complex). Basing off AdaFruit's python implementation at
 * https://github.com/adafruit/Adafruit_CircuitPython_VL53L0X/blob/main/adafruit_vl53l0x.py
 * 
 */
    public VL53L0X() {
        i2c = new I2C(Port.kMXP, i2c_addr);
        
        // Create simulation device and callbacks to prints the writes and reads
        // Use setSimBuffer to preload what reads should return (zeros by default)
        if (Robot.isSimulation()) {
            simDev = new I2CSim(Port.kMXP.value);
            simDev.setInitialized(true);
            BufferCallback readCallback = (String name, byte[] buffer, int count) -> {
                System.out.print(name + " " + count + " bytes:" ); 
                for (int i = 0; i < count; i++) {
                        buffer[i] = simBuf[i];
                        System.out.print(" " + String.format("%02X", buffer[i])); 
                }
                System.out.println();
                simBuf = new byte[simBufSize]; // Clear buffer after use
            };

            ConstBufferCallback writeCallback = (String name, byte[] buffer, int count) -> {
            System.out.print(name + " " + count + " bytes:" ); 
            for (int i = 0; i < count; i++) {
                    System.out.print(" " + String.format("%02X", buffer[i])); 
            }
            System.out.println();
            };
            readCallbackStore = simDev.registerReadCallback(readCallback);
            writeCallbackStore = simDev.registerWriteCallback(writeCallback);
        }

        readAndCheckListVL53L0X(
            List.of(
                new Pair<>(0xC0, 0xEE),
                new Pair<>(0xC1, 0xAA),
                new Pair<>(0xC2, 0x10)
            )
        );

    };

    public void setRange(int range) {
        range_mm = range;
        SmartDashboard.putNumber("ToF Range mm", range_mm);

    }

    public int getRange() {
        return range_mm;

    }

    public void setSimBuffer(byte[] buf) {
        for (int i = 0; i < buf.length; i++) {
            simBuf[i] = buf[i];
        }
    }
    public void setSimBuffer(int data, boolean byteNotWord) {
        if (byteNotWord) {
            simBuf[0] = (byte)(data & 0xFF);
        } else {
            simBuf[0] = (byte)((data & 0xFF00) >> 8);
            simBuf[1] = (byte)(data & 0xFF);
        }
    }

    public boolean isPresent() {
        return isPresent;
    }

    public static int mclksToMicroseconds(int mclks, int vcsel_period_pclks) {
        int macro_period_us = ((2304 * (vcsel_period_pclks) * 1655) + 500) / 1000;
        return (mclks * macro_period_us + macro_period_us/2)/1000;
    }


    public int read16VL53L0X(int index)
    {
        byte[] buf = readBufferVL53L0X(index, 2);
        return (((int) buf[0] & 0xFF) << 8) + ((int) buf[1] & 0xFF);
    }

    public byte[] readBufferVL53L0X(int index, int count) {
        byte[] buf = new byte[count];
        if (!isPresent || count > buf.length) {
            return null;
        }
        if (i2c.read(index,count,buf)){
            DriverStation.reportError("Communication error with Time of Flight sensor", true);
            isPresent = false;
            return null;
        }
        if (DEBUG) {
            System.out.print("Read from: "  + String.format("%02x", index) + 
                            "\nGot: ");
            
            for (int i = 0; i < count; i++) {
                    System.out.print(" " + String.format("%02X", buf[i])); 
            }
            System.out.println();
        }
        return buf;
    }


    public int readVL53L0X(int index) {
        byte[] buf = new byte[1];
        if (!isPresent) {
            return 0;
        }
        if (i2c.read(index,1,buf)){
            DriverStation.reportError("Communication error with Time of Flight sensor", true);
            isPresent = false;
            return 0;
        }
        if (DEBUG) {
            System.out.println(
                "Read from: "  + String.format("%02x", index) + 
                "\nGot: " + String.format("%02x", ((int) buf[0]) & 0xFF));
        }
        return ((int) buf[0]) & 0xFF;
    }
    
    public void readAndCheckListVL53L0X(List<Pair<Integer,Integer>> regPairs) {
        for (Pair<Integer,Integer> p : regPairs) {
            simBuf[0] = (byte)(p.getSecond() & 0xFF);
            int val = readVL53L0X(p.getFirst());
            if (val != p.getSecond()) {
                System.out.println(
                    "Read from " + String.format("%02x", p.getFirst()) + 
                    "\nExpected: " + String.format("%02x", p.getSecond()) + 
                    "\nGot: " + String.format("%02x", val));
                isPresent = false;
                break;
            }
        }
    }

    public void write16VL53L0X(int index, int data) {
        byte[] buf = {(byte)((data & 0xFF00) >> 8), (byte)(data & 0xFF)};
        writeBufferVL53L0X(index, buf);

    }

    public  void writeBufferVL53L0X(int index, byte[] buf) {
        byte[] writeBuf = new byte[buf.length+1];
        writeBuf[0] = (byte)index;
        for (int i = 1; i < writeBuf.length; i++) {
            writeBuf[1] = buf[i-1];
        }
        if (!isPresent) {
            return;
        }
        if (i2c.writeBulk(writeBuf)){
            DriverStation.reportError("Communication error with Time of Flight sensor", true);
            isPresent = false;
        }
    }

    public boolean writeVL53L0X(int index, int data) {
        if (!isPresent) {
            return true;
        }
        if (i2c.write(index,data)){
            DriverStation.reportError("Communication error with Time of Flight sensor", true);
            isPresent = false;
            return true;
        }
        return false;
    }

    public void writeListVL53L0X(List<Pair<Integer,Integer>> regPairs) {
        for (Pair<Integer,Integer> p : regPairs) {
            if(writeVL53L0X(p.getFirst(),p.getSecond())) {
                break;
            }
        }
    }

    @Override
    public void close() throws Exception {
        if (Robot.isSimulation()) {
            readCallbackStore.close();
            writeCallbackStore.close();
        }
        i2c.close();
    }
    

}
