    package frc.robot.hardware;

    import edu.wpi.first.wpilibj.PWM;

    /** REV Robotics Blinkin LED Driver. */
    //TODO PWM Bounds
    /*
     *Primarily for 5v driverIC with 3 wire interface (RGB) 
     *Can be made for 12v but might have some hardware limitations
     */
    public class LedDriver {
        private final PWM pwm;

        public LedDriver(int channel) { //Creates PWM channel for Led.
            pwm = new PWM(channel);
            pwm.setBounds(0, 0, 0, 0, 0);
            pwm.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
        }

        public void setMode(LedMode mode) {
            pwm.setSpeed(mode.value);
        }

            public static enum LedMode {
            CONE(-0.99), //PW 1005
            CUBE(-0.97); //PW 1015
            private final double value;

            LedMode(double value) {
                    this.value = value;
            }
        }
    }
