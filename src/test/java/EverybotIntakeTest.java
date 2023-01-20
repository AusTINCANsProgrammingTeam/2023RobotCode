import static org.junit.jupiter.api.Assertions.*; 


import edu.wpi.first.hal.HAL;

import org.junit.jupiter.api.*;

import frc.robot.subsystems.EverybotIntakeSubsystem;

// Extremely simple unit testing as an example of JUnit tests
// See https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/unit-testing.html for a slightly more detailed example
public class EverybotIntakeTest {

  private static final double kDelta = 1e-2;

  //DUT -> Device Under Test
  private EverybotIntakeSubsystem dut2;

  @BeforeEach // this method will run before each test
  public void setup() {
    //assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    dut2 = new EverybotIntakeSubsystem();
  }

  @AfterEach // this method will run after each test
  public void shutdown() throws Exception {
    dut2.close(); // destroy our subsystem object so we do each test from a clean slate
  }

  @Test // marks this method as a test
  public void testPull2() {
    dut2.pull();
    //JUnit Assert class provides this and many other comparision functions.
    assertEquals(EverybotIntakeSubsystem.kIntakeSpeed, dut2.getSpeed(), kDelta); //Check that motor speed matches what we wrote
  }
  @Test
  public void testPush2() {
    dut2.push();
    //JUnit Assert class provides this and many other comparision functions.
    assertEquals(EverybotIntakeSubsystem.kOuttakeSpeed, dut2.getSpeed(), kDelta);
  }
  // Add more @Test tagged methods to test other things...
}