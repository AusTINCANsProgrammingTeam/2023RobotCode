// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import edu.wpi.first.hal.simulation.BufferCallback;
import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.simulation.CallbackStore;
import edu.wpi.first.wpilibj.simulation.I2CSim;

import java.util.Map;

/** Add your docs here. */
public class VL53L0X {
    private static final int i2c_addr = 0x52;
    private final I2C i2c; 

    private I2CSim simDev;

    private static final int SYSRANGE_START = 0x00;
    private static final int SYSTEM_THRESH_HIGH = 0x0C;
    private static final int SYSTEM_THRESH_LOW = 0x0E;
    private static final int SYSTEM_SEQUENCE_CONFIG = 0x01;
    private static final int SYSTEM_RANGE_CONFIG = 0x09;
    private static final int SYSTEM_INTERMEASUREMENT_PERIOD = 0x04;
    private static final int SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A;
    private static final int GPIO_HV_MUX_ACTIVE_HIGH = 0x84;
    private static final int SYSTEM_INTERRUPT_CLEAR = 0x0B;
    private static final int RESULT_INTERRUPT_STATUS = 0x13;
    private static final int RESULT_RANGE_STATUS = 0x14;
    private static final int RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC;
    private static final int RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = 0xC0;
    private static final int RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0;
    private static final int RESULT_CORE_RANGING_TOTAL_EVENTS_REF = 0xD4;
    private static final int RESULT_PEAK_SIGNAL_RATE_REF = 0xB6;
    private static final int ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28;
    private static final int I2C_SLAVE_DEVICE_ADDRESS = 0x8A;
    private static final int MSRC_CONFIG_CONTROL = 0x60;
    private static final int PRE_RANGE_CONFIG_MIN_SNR = 0x27;
    private static final int PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56;
    private static final int PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57;
    private static final int PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64;
    private static final int FINAL_RANGE_CONFIG_MIN_SNR = 0x67;
    private static final int FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47;
    private static final int FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48;
    private static final int FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44;
    private static final int PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61;
    private static final int PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62;
    private static final int PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50;
    private static final int PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51;
    private static final int PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52;
    private static final int SYSTEM_HISTOGRAM_BIN = 0x81;
    private static final int HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33;
    private static final int HISTOGRAM_CONFIG_READOUT_CTRL = 0x55;
    private static final int FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70;
    private static final int FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71;
    private static final int FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72;
    private static final int CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20;
    private static final int MSRC_CONFIG_TIMEOUT_MACROP = 0x46;
    private static final int SOFT_RESET_GO2_SOFT_RESET_N = 0xBF;
    private static final int IDENTIFICATION_MODEL_ID = 0xC0;
    private static final int IDENTIFICATION_REVISION_ID = 0xC2;
    private static final int OSC_CALIBRATE_VAL = 0xF8;
    private static final int GLOBAL_CONFIG_VCSEL_WIDTH = 0x32;
    private static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0;
    private static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0xB1;
    private static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0xB2;
    private static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0xB3;
    private static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0xB4;
    private static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0xB5;
    private static final int GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6;
    private static final int DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E;
    private static final int DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F;
    private static final int POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80;
    private static final int VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89;
    private static final int ALGO_PHASECAL_LIM = 0x30;
    private static final int ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30;
    private static final int VCSEL_PERIOD_PRE_RANGE = 0;
    private static final int VCSEL_PERIOD_FINAL_RANGE = 1;

    private boolean isPresent = true;

    private static final Map<Integer, Integer> refRegs = Map.of(
        0xC0, 0xEE,
        0xC1, 0xAA,
        0xC2, 0x10

    );

    private int stopVariable;
    private CallbackStore readCallbackStore, writeCallbackStore;

/*
 * Vendor of this IC doesn't provide a register map (apparently its extremely complex). Basing off AdaFruit's python implementation
 */
    public VL53L0X() {
        i2c = new I2C(Port.kMXP, i2c_addr);
        
        // Simulation setup
        simDev = new I2CSim(Port.kMXP.value);
        simDev.setInitialized(true);
        BufferCallback readCallback = (String name, byte[] buffer, int count) -> {
           System.out.println(name + " " + count); 
        };

        ConstBufferCallback writeCallback = (String name, byte[] buffer, int count) -> {
           System.out.println(name + " " + count); 
        };
        readCallbackStore = simDev.registerReadCallback(readCallback);
        writeCallbackStore = simDev.registerWriteCallback(writeCallback);
        // End simulation setup

        readAndCheckMapVL53L0X(refRegs);
        // Initialize access to the sensor.  This is based on the logic from:
        //   https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp
        // Set I2C standard mode.
        writeMapVL53L0X(
            Map.of(
                0x88, 0x00,
                0x80, 0x01,
                0xFF, 0x01,
                0x00, 0x00
            )
        );
        stopVariable = readVL53L0X(0x91);
        writeMapVL53L0X(
            Map.of(
                0x00, 0x01,
                0xFF, 0x00,
                0x80, 0x00
            )
        );

        // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4)
        // limit checks
        int configControl = readVL53L0X(MSRC_CONFIG_CONTROL) | 0x12;
        writeVL53L0X(MSRC_CONFIG_CONTROL, configControl);

        // set final range signal rate limit to 0.25 MCPS (million counts per
        // second)
        double signalRateLimit = 0.25;
        writeVL53L0X(SYSTEM_SEQUENCE_CONFIG, 0xFF);

        //spad_count, spad_is_aperture = self._get_spad_info()

        // Get reference SPAD count and type, returned as a 2-tuple of
        // count and boolean is_aperture.  Based on code from:
        //   https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp

        writeMapVL53L0X(
            Map.of(
                0x80, 0x01,
                0xFF, 0x01,
                0x00, 0x00,
                0xFF, 0x06
            )
        );
        writeVL53L0X(0x83, readVL53L0X(0x83) | 0x04);
        writeMapVL53L0X(
            Map.of(
                0xFF, 0x07,
                0x81, 0x01,
                0x80, 0x01,
                0x94, 0x6B,
                0x83, 0x00
            )
        );

        //TODO Remove busy wait
        while(readVL53L0X(0x83) == 0x00) { }
        writeVL53L0X(0x83,0x01);
        int tmp = readVL53L0X(0x92);
        int spad_count = tmp & 0x7F;
        boolean spad_is_aperture = (tmp & 0x80) != 0;

        writeMapVL53L0X(
            Map.of(
                0x81, 0x00, 
                0xFF, 0x06
            )
        );
        writeVL53L0X(0x83, readVL53L0X(0x83) & 0xFB);

        writeMapVL53L0X(
            Map.of(
                0xFF, 0x01,
                0x00, 0x01, 
                0xFF, 0x00, 
                0x80, 0x00
            )
        );

        // The SPAD map (RefGoodSpadMap) is read by
        // VL53L0X_get_info_from_device() in the API, but the same data seems to
        // be more easily readable from GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through
        // _6, so read it from there.
        byte[] ref_spad_map = readBufferVL53L0X(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6);

        writeMapVL53L0X(
            Map.of(
                0xFF, 0x01,
                DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00,
                DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C,
                0xFF, 0x00,
                GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4
            )
        );

        int first_spad_to_enable =  spad_is_aperture ? 12 : 0;
        int spads_enabled = 0;
        for (int i = 0 ; i < 48; i++) {
            if (i < first_spad_to_enable || spads_enabled == spad_count) {
                // This bit is lower than the first one that should be enabled,
                // or (reference_spad_count) bits have already been enabled, so
                // zero this bit.
                ref_spad_map[1 + (i / 8)] &= ~(1 << (i % 8));
            } else if (((ref_spad_map[1 + (i / 8)] >> (i % 8)) & 0x01) > 0) {
                spads_enabled++;
            }
        }
        writeBufferVL53L0X(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map);
        writeMapVL53L0X(
            Map.of(
                0xFF, 0x01,
                0x00, 0x00,
                0xFF, 0x00,
                0x09, 0x00,
                0x10, 0x00,
                0x11, 0x00,
                0x24, 0x01,
                0x25, 0xFF,
                0x75, 0x00,
                0xFF, 0x01
            )
        );
        writeMapVL53L0X(
            Map.of(
                0x4E, 0x2C,
                0x48, 0x00,
                0x30, 0x20,
                0xFF, 0x00,
                0x30, 0x09,
                0x54, 0x00,
                0x31, 0x04,
                0x32, 0x03,
                0x40, 0x83,
                0x46, 0x25
            )
        );
        writeMapVL53L0X(
            Map.of(
                0x60, 0x00,
                0x27, 0x00,
                0x50, 0x06,
                0x51, 0x00,
                0x52, 0x96,
                0x56, 0x08,
                0x57, 0x30,
                0x61, 0x00,
                0x62, 0x00,
                0x64, 0x00
            )
        );
        writeMapVL53L0X(
            Map.of(
                0x65, 0x00,
                0x66, 0xA0,
                0xFF, 0x01,
                0x22, 0x32,
                0x47, 0x14,
                0x49, 0xFF,
                0x4A, 0x00,
                0xFF, 0x00,
                0x7A, 0x0A,
                0x7B, 0x00
            )
        );
        writeMapVL53L0X(
            Map.of(
                0x78, 0x21,
                0xFF, 0x01,
                0x23, 0x34,
                0x42, 0x00,
                0x44, 0xFF,
                0x45, 0x26,
                0x46, 0x05,
                0x40, 0x40,
                0x0E, 0x06,
                0x20, 0x1A
            )
        );
        writeMapVL53L0X(
            Map.of(
                0x43, 0x40,
                0xFF, 0x00,
                0x34, 0x03,
                0x35, 0x44,
                0xFF, 0x01,
                0x31, 0x04,
                0x4B, 0x09,
                0x4C, 0x05,
                0x4D, 0x04,
                0xFF, 0x00
            )
        );
        writeMapVL53L0X(
            Map.of(
                0x44, 0x00,
                0x45, 0x20,
                0x47, 0x08,
                0x48, 0x28,
                0x67, 0x00,
                0x70, 0x04,
                0x71, 0x01,
                0x72, 0xFE,
                0x76, 0x00,
                0x77, 0x00
            )
        );
        writeMapVL53L0X(
            Map.of(
                0xFF, 0x01,
                0x0D, 0x01,
                0xFF, 0x00,
                0x80, 0x01,
                0x01, 0xF8,
                0xFF, 0x01,
                0x8E, 0x01,
                0x00, 0x01,
                0xFF, 0x00,
                0x80, 0x00
            )
        );
        writeVL53L0X(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
        int gpio_hv_mux_active_high = readVL53L0X(GPIO_HV_MUX_ACTIVE_HIGH);
        writeVL53L0X(GPIO_HV_MUX_ACTIVE_HIGH, gpio_hv_mux_active_high & 0x7F);
        writeVL53L0X(SYSTEM_INTERRUPT_CLEAR, 0x01);


        //self._measurement_timing_budget_us = self.measurement_timing_budget
        int measurement_timing_budget = 1910 + 960; // Start overhead + end overhead

        // Get Sequence Step Enables
        int sequence_config = readVL53L0X(SYSTEM_SEQUENCE_CONFIG);
        boolean tcc = ((sequence_config >> 4) & 0x1) > 0;
        boolean dss = ((sequence_config >> 3) & 0x1) > 0;
        boolean msrc = ((sequence_config >> 2) & 0x1) > 0;
        boolean pre_range = ((sequence_config >> 6) & 0x1) > 0;
        boolean final_range = ((sequence_config >> 7) & 0x1) > 0;

        // Get Sequence Step Timeouts
        // based on get_sequence_step_timeout() from ST API but modified by
        // pololu here:
        //   https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp

        //pre_range_vcsel_period_pclks = self._get_vcsel_pulse_period( _VCSEL_PERIOD_PRE_RANGE)
        int pre_range_vcsel_period_pclks = ((readVL53L0X(PRE_RANGE_CONFIG_VCSEL_PERIOD)+1) & 0xFF) << 1;
        int msrc_dss_tcc_mclks = (readVL53L0X(MSRC_CONFIG_TIMEOUT_MACROP) + 1) & 0xFF;
        int msrc_dss_tcc_us = mclksToMicroseconds(msrc_dss_tcc_mclks, pre_range_vcsel_period_pclks);

        int pre_range_mclks_reg = read16VL53L0X(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI);
        double pre_range_mclks = (pre_range_mclks_reg & 0xFF) * Math.pow(2.0, (pre_range_mclks_reg & 0xFF00) >> 8) + 1;

        int pre_range_us = mclksToMicroseconds((int)pre_range_mclks, pre_range_vcsel_period_pclks);

        int final_range_vcsel_period_pclks = ((readVL53L0X(FINAL_RANGE_CONFIG_VCSEL_PERIOD)+1) & 0xFF) << 1;

        int final_range_mclks_reg = read16VL53L0X(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI);
        double final_range_mclks = (final_range_mclks_reg & 0xFF) * Math.pow(2.0, (final_range_mclks_reg & 0xFF00) >> 8) + 1;

        if (pre_range) {
            final_range_mclks -= pre_range_mclks;
        }
        int final_range_us = mclksToMicroseconds((int)final_range_mclks, final_range_vcsel_period_pclks);

        // Calculate measurement timing budget in microseconds
        if (tcc) {
            measurement_timing_budget += msrc_dss_tcc_us + 590;
        }
        if (dss) {
            measurement_timing_budget += 2 * (msrc_dss_tcc_us + 690);
        } else if (msrc) {
            measurement_timing_budget += (msrc_dss_tcc_us + 660);
        }
        if(pre_range) {
            measurement_timing_budget += pre_range_us + 660;
        }
        if (final_range) {
            measurement_timing_budget += final_range_us + 550;
        }


        writeVL53L0X(SYSTEM_SEQUENCE_CONFIG, 0xE8);

        //self.measurement_timing_budget = self._measurement_timing_budget_us
        if (measurement_timing_budget < 20000) {
            DriverStation.reportError("Time of Flight sensor timing budget error", true);
        }

        int used_budget_us = 1320 + 960; // Start (diff from get) + end overhead
        if (tcc) {
            used_budget_us += msrc_dss_tcc_us + 590;
        }
        if (dss) {
            used_budget_us += 2 * (msrc_dss_tcc_us + 690);
        } else if (msrc) {
            used_budget_us += (msrc_dss_tcc_us + 660);
        }
        if(pre_range) {
            used_budget_us += pre_range_us + 660;
        }
        if (final_range) {
            used_budget_us += final_range_us + 550;
            if (used_budget_us > measurement_timing_budget) {
                DriverStation.reportError("Time of Flight sensor timing budget error", true);
            }
            int final_range_timeout_us = measurement_timing_budget - used_budget_us;
            int final_range_timeout_mclks = mclksToMicroseconds(final_range_timeout_us, final_range_vcsel_period_pclks);
            if (pre_range) {
                final_range_timeout_mclks += pre_range_mclks;
            }
            int encoded_timeout = final_range_timeout_mclks & 0xFFFF;
            int ls_byte = 0;
            int ms_byte = 0;
            if (encoded_timeout > 0) {
                ls_byte = encoded_timeout -1;
                while (ls_byte > 255) {
                    ls_byte >>= 1;
                    ms_byte++;
                }
                encoded_timeout = ((ms_byte << 8) | (ls_byte & 0xFF)) & 0xFFFF;
            }
            write16VL53L0X(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encoded_timeout);
        }



        writeVL53L0X(SYSTEM_SEQUENCE_CONFIG, 0x01);

        performSingleRefCal(0x40);
        writeVL53L0X(SYSTEM_SEQUENCE_CONFIG, 0x02);

        performSingleRefCal(0x00);
        // "restore the previous Sequence Config"
        writeVL53L0X(SYSTEM_SEQUENCE_CONFIG, 0xE8);

    };

    public int getRange() {
        writeMapVL53L0X(
            Map.of(
                0x80, 0x01,
                0xFF, 0x01,
                0x00, 0x00,
                0x91, stopVariable,
                0x00, 0x01,
                0xFF, 0x00,
                0x80, 0x00,
                SYSRANGE_START, 0x01
            )
        );

        //TODO Fix busy wait x2
        while ((readVL53L0X(SYSRANGE_START) & 0x01) > 0) {}
        while ((readVL53L0X(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {}
        int rangeBuf = read16VL53L0X(RESULT_RANGE_STATUS + 10);
        writeVL53L0X(SYSTEM_INTERRUPT_CLEAR, 0x01);
        return rangeBuf;

    }

    private int mclksToMicroseconds(int mclks, int vcsel_period_pclks) {
        int macro_period_us = ((2304 * (vcsel_period_pclks) * 1655) + 500) / 1000;
        return (mclks * macro_period_us + macro_period_us/2)/1000;
    }

    private void performSingleRefCal(int vhv_init_byte) {
        // based on VL53L0X_perform_single_ref_calibration() from ST API.
        writeVL53L0X(SYSRANGE_START, 0x01 | vhv_init_byte & 0xFF);
        // TODO Fix busy wait
        while((readVL53L0X(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {}
        writeVL53L0X(SYSTEM_INTERRUPT_CLEAR, 0x01);
        writeVL53L0X(SYSRANGE_START, 0x00);
    }

    private int read16VL53L0X(int index)
    {
        byte[] buf = readBufferVL53L0X(index, 2);
        return (((int) buf[0] & 0xFF) << 8) + ((int) buf[1] & 0xFF);
    }

    private byte[] readBufferVL53L0X(int index, int count) {
        byte[] buf = new byte[count];
        if (!isPresent || count > buf.length) {
            return null;
        }
        if (i2c.read(index,count,buf)){
            DriverStation.reportError("Communication error with Time of Flight sensor", true);
            isPresent = false;
            return null;
        }
        return buf;
    }


    private int readVL53L0X(int index) {
        byte[] buf = new byte[1];
        if (!isPresent) {
            return 0;
        }
        if (i2c.read(index,1,buf)){
            DriverStation.reportError("Communication error with Time of Flight sensor", true);
            isPresent = false;
            return 0;
        }
        return ((int) buf[0]) & 0xFF;
    }
    
    private void readAndCheckMapVL53L0X(Map<Integer,Integer> regPairs) {
        for (Integer k : regPairs.keySet()) {
            if (readVL53L0X(k) != refRegs.get(k).intValue()) {
                isPresent = false;
                break;
            }
        }
    }

    private void write16VL53L0X(int index, int data) {
        byte[] buf = {(byte)((data & 0xFF00) >> 8), (byte)(data & 0xFF)};
        writeBufferVL53L0X(index, buf);

    }

    private void writeBufferVL53L0X(int index, byte[] buf) {
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

    private boolean writeVL53L0X(int index, int data) {
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
    private void writeMapVL53L0X(Map<Integer,Integer> regPairs) {
        for (Integer k : regPairs.keySet()) {
            if(writeVL53L0X(k.intValue(),regPairs.get(k))) {
                break;
            }
        }
    }
    

}
