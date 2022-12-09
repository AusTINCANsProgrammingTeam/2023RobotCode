package frc.robot.hardware;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.util.Units;

public class AbsoluteEncoder {
    public enum EncoderConfig {
        //Swerve Modules
        FrontLeftModule(11, false, 4.23),
        FrontRightModule(12, true, 4.25),
        BackLeftModule(9, false, 1.57),
        BackRightModule(10, true, 3.91);
        
        private int ID;
        private boolean reversed;
        private double offset; //Offset in radians
    
        EncoderConfig(int ID, boolean reversed, double offset){
            this.ID = ID;
            this.reversed = reversed;
            this.offset = offset;
        }
    
        EncoderConfig(int ID, boolean reversed){
            this(ID, reversed, 0);
        }
    
        EncoderConfig(int ID){
            this(ID, false, 0);
        }
    
        public int getID(){
            return ID;
        }

        public boolean getReversed(){
            return reversed;
        }

        public double getOffset(){
            return offset;
        }
    }

    public static WPI_CANCoder constructEncoder(EncoderConfig config){
        WPI_CANCoder encoder = new WPI_CANCoder(config.getID());
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        encoder.configSensorDirection(config.getReversed());
        encoder.configMagnetOffset(Units.radiansToDegrees(config.getOffset()));
        return encoder;
    }
}
