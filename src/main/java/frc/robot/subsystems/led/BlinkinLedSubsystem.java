package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
// TODO Led parameters and RIO ports
public class BlinkinLedSubsystem extends SubsystemBase {
    private BlinkinLedMode mode = BlinkinLedMode.SOLID_YELLOW;
    private final Spark spark;

    public static enum BlinkinLedMode {
        SOLID_YELLOW(0.69), SOLID_VIOLET(0.91), OFF(0.99);
    private final double value;
        BlinkinLedMode(double value) {
            this.value = value;
        }
    }
    
    public BlinkinLedSubsystem() {
        spark = new Spark(Robot.ledStipPort);
    }

    public void blinkinStopLed(){
        spark.set(BlinkinLedMode.OFF.value);
    }

    public void blinkinStartLed(){
        spark.set(mode.value);
    }

    public void blinkinChangeGamePiece(){
        if (mode == BlinkinLedMode.SOLID_YELLOW){
            mode = BlinkinLedMode.SOLID_VIOLET;
            spark.set(mode.value);
        } else if (mode == BlinkinLedMode.SOLID_VIOLET){
            mode = BlinkinLedMode.SOLID_YELLOW;
            spark.set(mode.value);
        }
    }


    @Override
    public void periodic() {
    }
}
