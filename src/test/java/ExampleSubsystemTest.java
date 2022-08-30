import static org.junit.Assert.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.*;

import java.util.Random;

import org.junit.*;

import frc.robot.subsystems.ExampleSubsystem;


// Extremely simple unit testing as an example of JUnit tests
// See https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/unit-testing.html for a slightly more detailed example
public class ExampleSubsystemTest {

  private static final double kDelta = 1e-2;
  private static final int kNumTests = 10;

  //DUT -> Device Under Test
  private ExampleSubsystem dut;

  private PWMSim simMotor;

  @Before // this method will run before each test
  public void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    dut = new ExampleSubsystem();
    simMotor = new PWMSim(0); // create our simulation PWM motor controller as an example
  }

  @After // this method will run after each test
  public void shutdown() throws Exception {
    dut.close(); // destroy our subsystem object so we do each test from a clean slate
    simMotor.resetData();
  }

  @Test // marks this method as a test
  public void testSpeed() {
    Random rand = new Random();
    for (int i = 0; i < kNumTests; i++){
        // nextDouble gets random value between 0.0 and 1.0. Do math to extend the range and center on 0.
        double s = 2*rand.nextDouble()-1;
        dut.spin(s);
        //JUnit Assert class provides this and many other comparision functions.
        assertEquals(s, simMotor.getSpeed(), kDelta); //Check that motor speed matches what we wrote
    }
  }

  // Add more @Test tagged methods to test other things...
}
