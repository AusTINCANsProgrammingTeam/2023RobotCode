// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

import java.util.Map;

/** Add your docs here. */
public class VL53L0X {
    private static final int i2c_addr = 0x52;
    private final I2C i2c; 

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
    private byte[] buf = new byte[128];

    private boolean isPresent = true;

    private static final Map<Integer, Integer> refRegs = Map.of(
        0xC0, 0xEE,
        0xC1, 0xAA,
        0xC2, 0x10

    );

    private static final Map<Integer, Integer> i2cModeRegs = Map.of(
        0x88, 0x00,
        0x80, 0x01,
        0xFF, 0x01,
        0x00, 0x00

    );

/*
 * Vendor of this IC doesn't provide a register map (apparently its extremely complex). Basing off AdaFruit's python implementation
 */
    public VL53L0X() {
        i2c = new I2C(Port.kMXP, i2c_addr);
        for (Integer k : refRegs.keySet()) {
            if (!i2c.read(k.intValue(),1,buf) || buf[0] != (byte)refRegs.get(k).intValue()) {
                DriverStation.reportError("Communication error with Time of Flight sensor", true);
                isPresent = false;
                break;
            }
        }

    };
    

}
