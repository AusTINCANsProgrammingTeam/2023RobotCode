import static org.junit.jupiter.api.Assertions.*; 
import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.*;

import frc.robot.classes.TimeOfFlightSensor;
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