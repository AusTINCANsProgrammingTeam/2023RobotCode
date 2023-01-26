package frc.robot.hardware;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

/** REV Robotics Blinkin LED Driver. */
public class BlinkinDriver {
    private final Spark spark;
    //TODO setbounds
    public BlinkinDriver(int channel) {
        spark = new Spark(channel);
    }

    public void setMode(BlinkinLedMode mode) {
        spark.set(mode.value);
    }

    public static enum BlinkinLedMode {
        SOLID_YELLOW(0.69), SOLID_VIOLET(0.91), OFF(0);

    private final double value;

        BlinkinLedMode(double value) {
            this.value = value;
        }
    }
}