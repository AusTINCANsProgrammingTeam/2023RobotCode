import static org.junit.jupiter.api.Assertions.*; 

import org.junit.jupiter.api.*;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.BuddyBalanceSubsystem;

// Extremely simple unit testing as an example of JUnit tests
// See https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/unit-testing.html for a slightly more detailed example
public class BuddyBalanceTest {

  private static final double kDelta = 1e-2;

  //DUT -> Device Under Test
  private BuddyBalanceSubsystem dut;
  @BeforeEach // this method will run before each test
  public void setup() {
    dut = new BuddyBalanceSubsystem();
  assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
  }

  @AfterEach // this method will run after each test
  public void shutdown() throws Exception {
    dut.close(); // destroy our subsystem object so we do each test from a clean slate
  }

  @Test
  public void testDeploy() {
    dut.deployBuddyBalance();
    //JUnit Assert class provides this and many other comparision functions.
    assertEquals(BuddyBalanceSubsystem.kServoDeployedPos, dut.getServoAngle(), kDelta); //Check that motor speed matches what we wrote
  }

  @Test
  public void testRetrieve() {
    dut.retrieveBuddy();
    //JUnit Assert class provides this and many other comparision functions.
    assertEquals(BuddyBalanceSubsystem.kRetrievedAngle, dut.getEncoderRightAngle(), kDelta); //Check that right encoder angle matches what we wrote
    assertEquals(BuddyBalanceSubsystem.kRetrievedAngle, dut.getEncoderLeftAngle(), kDelta); //Check that right encoder angle matches what we wrote
  }
  @Test
  public void testRelease() {
    dut.releaseBuddy();
    //JUnit Assert class provides this and many other comparision functions.
    assertEquals(BuddyBalanceSubsystem.kDeployedAngle, dut.getEncoderRightAngle(), kDelta); //Check that right encoder angle matches what we wrote
    assertEquals(BuddyBalanceSubsystem.kDeployedAngle, dut.getEncoderLeftAngle(), kDelta); //Check that left encoder angle matches what we wrote
  }
  // Add more @Test tagged methods to test other things...
}