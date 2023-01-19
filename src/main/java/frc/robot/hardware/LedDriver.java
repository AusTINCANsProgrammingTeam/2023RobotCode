package frc.robot.hardware;

import edu.wpi.first.wpilibj.PWM;

/** REV Robotics Blinkin LED Driver. */
public class LedDriver {
  private final PWM pwm;

  public LedDriver(int channel) {
    pwm = new PWM(channel);
    pwm.setBounds(0, 0, 0, 0, 0);
    pwm.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
  }

  public void setMode(LedMode mode) {
    pwm.setSpeed(mode.value);
  }

  public static enum LedMode {
    CONE(-0.99),
    CUBE(-0.97);
    private final double value;

    LedMode(double value) {
      this.value = value;
    }
  }
}
