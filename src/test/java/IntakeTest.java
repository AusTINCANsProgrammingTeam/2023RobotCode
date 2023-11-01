/*!
 * Copyright (c) FIRST and other WPILib contributors.
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 * 
 * @file IntakeTest.java
 *
 * @brief Test class for the intake
 *
 * @section Changelog
 * Co-authored-by: JP Cassar <jp@cassartx.net>
 * Corrected outstanding errors
 */

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.IntakeSubsystem;


// Extremely simple unit testing as an example of JUnit tests
// See https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/unit-testing.html for a slightly more detailed example
public class IntakeTest {

  private static final double kDelta = 1e-2;

  //DUT -> Device Under Test
  private IntakeSubsystem dut;

  @BeforeEach // this method will run before each test
  public void setup() {
    dut = new IntakeSubsystem();
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
  }

  @AfterEach // this method will run after each test
  public void shutdown() throws Exception {
    dut.close(); // destroy our subsystem object so we do each test from a clean slate
  }
   
  
  @Test // marks this method as a test
  public void testPull() {
    dut.pull();
    //JUnit Assert class provides this and many other comparision functions.
    assertEquals(IntakeSubsystem.kConeIntakeSpeed, dut.getSpeed(), kDelta); //Check that motor speed matches what we wrote
  }
  @Test
  public void testPush() {
    dut.push();
    //JUnit Assert class provides this and many other comparision functions.
    assertEquals(IntakeSubsystem.kConeOuttakeSpeed, dut.getSpeed(), kDelta);
  }
  // Add more @Test tagged methods to test other things...
}