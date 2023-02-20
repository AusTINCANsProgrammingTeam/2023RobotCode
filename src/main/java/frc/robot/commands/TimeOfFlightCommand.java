// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.VL53L0X;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TimeOfFlightCommand extends SequentialCommandGroup {
  private int stopVariable;
  private final boolean BYTE = true;
  private final boolean WORD = false;
  private int configControl, spad_count;
  private int signalRateLimit = (int)(0.25 * (1 << 7));
  private int tmp;
  private boolean spad_is_aperture;
  private byte[] ref_spad_map;
  private int first_spad_to_enable;
  private int spads_enabled;
  private int gpio_hv_mux_active_high;

  private int measurement_timing_budget;
  private int sequence_config;
  private boolean tcc;
  private boolean dss;
  private boolean msrc;
  private boolean pre_range;
  private boolean final_range;

  private int pre_range_vcsel_period_pclks;
  private int msrc_dss_tcc_mclks;
  private int msrc_dss_tcc_us;
  private int pre_range_mclks_reg;
  private double pre_range_mclks;
  private int pre_range_us;
  private int final_range_vcsel_period_pclks;
  private int final_range_mclks_reg;
  private double final_range_mclks;
  private int encoded_timeout = 0; 
  private int final_range_us;


  /** Creates a new TimeOfFlightCommand. */
  public TimeOfFlightCommand(VL53L0X tof) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new I2cWriteCommand(
        tof, 
        List.of(
            new Pair<Integer,Integer>(0x88, 0x00),
            new Pair<Integer,Integer>(0x80, 0x01),
            new Pair<Integer,Integer>(0xFF, 0x01),
            new Pair<Integer,Integer>(0x00, 0x00)
        ),
        BYTE
      ),
      new I2cReadCommand(tof, (a) -> {stopVariable = a;}, 0x91, BYTE),
      new I2cWriteCommand(
        tof, 
        List.of(
            new Pair<Integer,Integer>(0x00, 0x01),
            new Pair<Integer,Integer>(0xFF, 0x00),
            new Pair<Integer,Integer>(0x80, 0x00)
        ),
        BYTE
      ),
      new I2cReadCommand(tof, (a) -> {configControl = a | 0x12;}, VL53L0X.MSRC_CONFIG_CONTROL, BYTE),
      new I2cWriteCommand(tof, VL53L0X.MSRC_CONFIG_CONTROL, configControl, BYTE),
      new I2cWriteCommand(tof, VL53L0X.FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, signalRateLimit, WORD),
      new I2cWriteCommand(tof, VL53L0X.SYSTEM_SEQUENCE_CONFIG, 0xFF, BYTE),
      new I2cWriteCommand(
        tof, 
        List.of(
            new Pair<Integer,Integer>(0x80, 0x01),
            new Pair<Integer,Integer>(0xFF, 0x01),
            new Pair<Integer,Integer>(0x00, 0x00),
            new Pair<Integer,Integer>(0xFF, 0x06)
        ),
        BYTE
      ),
      new I2cReadCommand(tof, (a) -> {tmp = a | 0x04;}, 0x83, BYTE),
      new I2cWriteCommand(tof, 0x83, () -> { return tmp; }, BYTE),
      new I2cWriteCommand(
        tof, 
        List.of(
            new Pair<Integer,Integer>(0xFF, 0x07),
            new Pair<Integer,Integer>(0x81, 0x01),
            new Pair<Integer,Integer>(0x80, 0x01),
            new Pair<Integer,Integer>(0x94, 0x6B),
            new Pair<Integer,Integer>(0x83, 0x00)
        ),
        BYTE
      ),
      new I2cPollCommand(tof, 0x83, 0xFF),
      new I2cWriteCommand(tof, 0x83, 0x01, BYTE),
      new I2cReadCommand(tof, 
        (a) -> {
          tmp = a;
          spad_count = tmp & 0x7F;
          spad_is_aperture = (tmp & 0x80) != 0;
        }, 
  0x92, 
        BYTE
      ),
      new I2cWriteCommand(
        tof, 
        List.of(
          new Pair<Integer,Integer>(0x81, 0x00), 
          new Pair<Integer,Integer>(0xFF, 0x06)
        ),
        BYTE
      ),
      new I2cReadCommand(tof, (a) -> {tmp = a & 0xFB;}, 0x83, BYTE),
      new I2cWriteCommand(tof, 0x83, () -> {return tmp;} , BYTE),
      new I2cWriteCommand(
        tof, 
        List.of(
          new Pair<Integer,Integer>(0xFF, 0x01),
          new Pair<Integer,Integer>(0x00, 0x01), 
          new Pair<Integer,Integer>(0xFF, 0x00), 
          new Pair<Integer,Integer>(0x80, 0x00)
        ),
        BYTE
      ),
      new I2cReadCommand(
        tof, 
        (a) -> {
          ref_spad_map = a;
          first_spad_to_enable =  spad_is_aperture ? 12 : 0;
          spads_enabled = 0;
          for (int i = 0 ; i < 48; i++) {
              if (i < first_spad_to_enable || spads_enabled == spad_count) {
                  // This bit is lower than the first one that should be enabled,
                  // or (reference_spad_count) bits have already been enabled, so
                  // zero this bit.
                  ref_spad_map[(i / 8)] &= ~(1 << (i % 8));
              } else if (((ref_spad_map[(i / 8)] >> (i % 8)) & 0x01) > 0) {
                  spads_enabled++;
              }
          }
        }, 
        VL53L0X.GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 
        6),
      new I2cWriteCommand(
        tof, 
        List.of(
          new Pair<Integer,Integer>(0xFF, 0x01),
          new Pair<Integer,Integer>(VL53L0X.DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00),
          new Pair<Integer,Integer>(VL53L0X.DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C),
          new Pair<Integer,Integer>(0xFF, 0x00),
          new Pair<Integer,Integer>(VL53L0X.GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4)
        ),
        BYTE
      ),
      new I2cWriteCommand(tof, VL53L0X.GLOBAL_CONFIG_SPAD_ENABLES_REF_0, () -> {return ref_spad_map;}),
      new I2cWriteCommand(
        tof, 
        List.of(
          new Pair<Integer,Integer>(0xFF, 0x01),
          new Pair<Integer,Integer>(0x00, 0x00),
          new Pair<Integer,Integer>(0xFF, 0x00),
          new Pair<Integer,Integer>(0x09, 0x00),
          new Pair<Integer,Integer>(0x10, 0x00),
          new Pair<Integer,Integer>(0x11, 0x00),
          new Pair<Integer,Integer>(0x24, 0x01),
          new Pair<Integer,Integer>(0x25, 0xFF),
          new Pair<Integer,Integer>(0x75, 0x00),
          new Pair<Integer,Integer>(0xFF, 0x01)
        ),
        BYTE
      ),
      new I2cWriteCommand(
        tof,
        List.of(
          new Pair<Integer,Integer>(0x4E, 0x2C),
          new Pair<Integer,Integer>(0x48, 0x00),
          new Pair<Integer,Integer>(0x30, 0x20),
          new Pair<Integer,Integer>(0xFF, 0x00),
          new Pair<Integer,Integer>(0x30, 0x09),
          new Pair<Integer,Integer>(0x54, 0x00),
          new Pair<Integer,Integer>(0x31, 0x04),
          new Pair<Integer,Integer>(0x32, 0x03),
          new Pair<Integer,Integer>(0x40, 0x83),
          new Pair<Integer,Integer>(0x46, 0x25)
        ),
        BYTE
      ),
      new I2cWriteCommand(
        tof,
        List.of(
          new Pair<Integer,Integer>(0x60, 0x00),
          new Pair<Integer,Integer>(0x27, 0x00),
          new Pair<Integer,Integer>(0x50, 0x06),
          new Pair<Integer,Integer>(0x51, 0x00),
          new Pair<Integer,Integer>(0x52, 0x96),
          new Pair<Integer,Integer>(0x56, 0x08),
          new Pair<Integer,Integer>(0x57, 0x30),
          new Pair<Integer,Integer>(0x61, 0x00),
          new Pair<Integer,Integer>(0x62, 0x00),
          new Pair<Integer,Integer>(0x64, 0x00)
        ),
        BYTE
      ),
      new I2cWriteCommand(
        tof,
        List.of(
          new Pair<Integer,Integer>(0x65, 0x00),
          new Pair<Integer,Integer>(0x66, 0xA0),
          new Pair<Integer,Integer>(0xFF, 0x01),
          new Pair<Integer,Integer>(0x22, 0x32),
          new Pair<Integer,Integer>(0x47, 0x14),
          new Pair<Integer,Integer>(0x49, 0xFF),
          new Pair<Integer,Integer>(0x4A, 0x00),
          new Pair<Integer,Integer>(0xFF, 0x00),
          new Pair<Integer,Integer>(0x7A, 0x0A),
          new Pair<Integer,Integer>(0x7B, 0x00)
        ),
        BYTE
      ),
      new I2cWriteCommand(
        tof,
          List.of(
            new Pair<Integer,Integer>(0x78, 0x21),
            new Pair<Integer,Integer>(0xFF, 0x01),
            new Pair<Integer,Integer>(0x23, 0x34),
            new Pair<Integer,Integer>(0x42, 0x00),
            new Pair<Integer,Integer>(0x44, 0xFF),
            new Pair<Integer,Integer>(0x45, 0x26),
            new Pair<Integer,Integer>(0x46, 0x05),
            new Pair<Integer,Integer>(0x40, 0x40),
            new Pair<Integer,Integer>(0x0E, 0x06),
            new Pair<Integer,Integer>(0x20, 0x1A)
          ),
        BYTE
      ),
      new I2cWriteCommand(
        tof,
        List.of(
          new Pair<Integer,Integer>(0x43, 0x40),
          new Pair<Integer,Integer>(0xFF, 0x00),
          new Pair<Integer,Integer>(0x34, 0x03),
          new Pair<Integer,Integer>(0x35, 0x44),
          new Pair<Integer,Integer>(0xFF, 0x01),
          new Pair<Integer,Integer>(0x31, 0x04),
          new Pair<Integer,Integer>(0x4B, 0x09),
          new Pair<Integer,Integer>(0x4C, 0x05),
          new Pair<Integer,Integer>(0x4D, 0x04),
          new Pair<Integer,Integer>(0xFF, 0x00)
        ),
        BYTE
      ),
      new I2cWriteCommand(
        tof,
        List.of(
          new Pair<Integer,Integer>(0x44, 0x00),
          new Pair<Integer,Integer>(0x45, 0x20),
          new Pair<Integer,Integer>(0x47, 0x08),
          new Pair<Integer,Integer>(0x48, 0x28),
          new Pair<Integer,Integer>(0x67, 0x00),
          new Pair<Integer,Integer>(0x70, 0x04),
          new Pair<Integer,Integer>(0x71, 0x01),
          new Pair<Integer,Integer>(0x72, 0xFE),
          new Pair<Integer,Integer>(0x76, 0x00),
          new Pair<Integer,Integer>(0x77, 0x00)
        ),
        BYTE
      ),
      new I2cWriteCommand(
        tof,
        List.of(
          new Pair<Integer,Integer>(0xFF, 0x01),
          new Pair<Integer,Integer>(0x0D, 0x01),
          new Pair<Integer,Integer>(0xFF, 0x00),
          new Pair<Integer,Integer>(0x80, 0x01),
          new Pair<Integer,Integer>(0x01, 0xF8),
          new Pair<Integer,Integer>(0xFF, 0x01),
          new Pair<Integer,Integer>(0x8E, 0x01),
          new Pair<Integer,Integer>(0x00, 0x01),
          new Pair<Integer,Integer>(0xFF, 0x00),
          new Pair<Integer,Integer>(0x80, 0x00)
        ),
        BYTE
      ),
      new I2cWriteCommand(tof, VL53L0X.SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04, BYTE),
      new I2cReadCommand(tof, (a) -> {gpio_hv_mux_active_high = a;}, VL53L0X.GPIO_HV_MUX_ACTIVE_HIGH, BYTE),
      new I2cWriteCommand(tof, VL53L0X.GPIO_HV_MUX_ACTIVE_HIGH, () -> {return gpio_hv_mux_active_high & 0x7F;}, BYTE),
      new I2cWriteCommand(tof, VL53L0X.SYSTEM_INTERRUPT_CONFIG_GPIO, 0x01, BYTE),
      new I2cReadCommand(
        tof, 
        (a) -> {
          sequence_config = a;
          measurement_timing_budget = 1910 + 960; // Start overhead + end overhead

          // Get Sequence Step Enables
          tcc = ((sequence_config >> 4) & 0x1) > 0;
          dss = ((sequence_config >> 3) & 0x1) > 0;
          msrc = ((sequence_config >> 2) & 0x1) > 0;
          pre_range = ((sequence_config >> 6) & 0x1) > 0;
          final_range = ((sequence_config >> 7) & 0x1) > 0;
        }, 
        VL53L0X.SYSTEM_SEQUENCE_CONFIG, 
        BYTE
      ),
      new I2cReadCommand(tof, (a) -> {pre_range_vcsel_period_pclks = ((a+1) & 0xFF) << 1;}, VL53L0X.PRE_RANGE_CONFIG_VCSEL_PERIOD, BYTE),
      new I2cReadCommand(
        tof, 
        (a) -> {
          msrc_dss_tcc_mclks = ((a+1) & 0xFF);
          msrc_dss_tcc_us = VL53L0X.mclksToMicroseconds(msrc_dss_tcc_mclks, pre_range_vcsel_period_pclks);
        },
        VL53L0X.MSRC_CONFIG_TIMEOUT_MACROP, 
        BYTE
      ),
      new I2cReadCommand(
        tof, 
        (a) -> {
          pre_range_mclks_reg = a;
          pre_range_mclks = (pre_range_mclks_reg & 0xFF) * Math.pow(2.0, (pre_range_mclks_reg & 0xFF00) >> 8) + 1;
          pre_range_us = VL53L0X.mclksToMicroseconds((int)pre_range_mclks, pre_range_vcsel_period_pclks);
        }, 
        VL53L0X.PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, 
        WORD
      ),
      new I2cReadCommand(tof, (a) -> {final_range_vcsel_period_pclks = ((a+1) & 0xFF) << 1;}, VL53L0X.FINAL_RANGE_CONFIG_VCSEL_PERIOD, BYTE),
      new I2cReadCommand(
        tof, 
        (a) -> {
          final_range_mclks_reg = a;
          final_range_mclks = (final_range_mclks_reg & 0xFF) * Math.pow(2.0, (final_range_mclks_reg & 0xFF00) >> 8) + 1;
          if (pre_range) {
              final_range_mclks -= pre_range_mclks;
          }
          final_range_us = VL53L0X.mclksToMicroseconds((int)final_range_mclks, final_range_vcsel_period_pclks);

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
        }, 
        VL53L0X.FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, 
        WORD
      ),
      new I2cWriteCommand(tof, VL53L0X.SYSTEM_SEQUENCE_CONFIG, 0xE8, BYTE),
      new InstantCommand(() -> {
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
            int final_range_timeout_mclks = VL53L0X.mclksToMicroseconds(final_range_timeout_us, final_range_vcsel_period_pclks);
            if (pre_range) {
                final_range_timeout_mclks += pre_range_mclks;
            }
            encoded_timeout = final_range_timeout_mclks & 0xFFFF;
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
          }
        }
      ),
      new ConditionalCommand(
        new I2cWriteCommand(tof, VL53L0X.FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, () -> {return encoded_timeout;}, BYTE), 
        new InstantCommand(() -> {}), 
        () -> {return final_range;}
      ),
      new I2cWriteCommand(tof, VL53L0X.SYSTEM_SEQUENCE_CONFIG, 0x01, BYTE), 
      getSingleRefCalCommand(tof, 0x40),
      new I2cWriteCommand(tof, VL53L0X.SYSTEM_SEQUENCE_CONFIG, 0x02, BYTE), 
      getSingleRefCalCommand(tof, 0x00),
      new I2cWriteCommand(tof, VL53L0X.SYSTEM_SEQUENCE_CONFIG, 0xE8, BYTE), 
      new RepeatCommand(getRangeCommand(tof))
    );
  }

  private Command getSingleRefCalCommand(VL53L0X tof, int vhv_init_byte) {
    return new SequentialCommandGroup(
      new I2cWriteCommand(tof, VL53L0X.SYSRANGE_START, () -> {return (0x01 | vhv_init_byte & 0xFF);}, BYTE), 
      new I2cPollCommand(tof, VL53L0X.RESULT_INTERRUPT_STATUS, 0x07),
      new I2cWriteCommand(tof, VL53L0X.SYSTEM_INTERRUPT_CLEAR, 0x01, BYTE), 
      new I2cWriteCommand(tof, VL53L0X.SYSRANGE_START, 0x00, BYTE)
    );
  }

  private Command getRangeCommand(VL53L0X tof) {
    return new SequentialCommandGroup(
      new I2cWriteCommand(
        tof,
        List.of(
            new Pair<Integer,Integer>(0x80, 0x01),
            new Pair<Integer,Integer>(0xFF, 0x01),
            new Pair<Integer,Integer>(0x00, 0x00)
        ),
        BYTE
      ),
      new I2cWriteCommand(tof, 0x91, () -> {return stopVariable;}, BYTE),
      new I2cWriteCommand(
        tof,
        List.of(
            new Pair<Integer,Integer>(0x00, 0x01),
            new Pair<Integer,Integer>(0xFF, 0x00),
            new Pair<Integer,Integer>(0x80, 0x00),
            new Pair<Integer,Integer>(VL53L0X.SYSRANGE_START, 0x01)
        ),
        BYTE
      ),
      new I2cPollCommand(tof, VL53L0X.SYSRANGE_START, 0x01),
      new I2cPollCommand(tof, VL53L0X.RESULT_INTERRUPT_STATUS, 0x07),
      new I2cReadCommand(tof, tof::setRange, VL53L0X.RESULT_RANGE_STATUS+10, WORD),
      new I2cWriteCommand(tof, VL53L0X.SYSTEM_INTERRUPT_CLEAR, 0x01, BYTE)
    );
  }

  
}
