package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.LedDriver;
import frc.robot.hardware.LedDriver.BlinkinLedMode;
// TODO Led parameters and RIO ports
public class BlinkinLedSubsystem extends SubsystemBase {
    private static final int port = 0; //TODO setport
    private final LedDriver blinkin;
    private BlinkinLedMode mode = BlinkinLedMode.SOLID_YELLOW;

    public BlinkinLedSubsystem() {
        blinkin = new LedDriver(port);
    }

    public void blinkinStopLed(){
        blinkin.setMode(BlinkinLedMode.OFF);
    }

    public void blinkinStartLed(){
        blinkin.setMode(mode);
    }

    public void blinkinChangeGamePiece(){
        if (mode == BlinkinLedMode.SOLID_YELLOW){
            mode = BlinkinLedMode.SOLID_VIOLET;
            blinkin.setMode(mode);
        } else if (mode == BlinkinLedMode.SOLID_VIOLET){
            mode = BlinkinLedMode.SOLID_YELLOW;
            blinkin.setMode(mode);
        }
    }


    @Override
    public void periodic() {
    }
}
