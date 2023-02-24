package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
public class BlinkinLedSubsystem extends SubsystemBase {
    private final Spark spark;

    public static enum BlinkinMode {
        BLINKIN_YELLOW(0.69), BLINKIN_PURPLE(0.91), BLINKIN_OFF(0.99);
        private final double value;
        BlinkinMode(double value) {
                this.value = value;
        }
    }
    
    public BlinkinLedSubsystem() {
        spark = new Spark(Robot.ledPort);
    }

    public void blinkinStopLed(){
        spark.set(BlinkinMode.BLINKIN_OFF.value);
    }

    public void cargoLed(BlinkinMode mode){
        spark.set(mode.value);
    }


    @Override
    public void periodic() {
    }
}
