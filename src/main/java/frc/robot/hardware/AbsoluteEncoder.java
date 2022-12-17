package frc.robot.hardware;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.util.Units;

public class AbsoluteEncoder {
    public enum EncoderConfig {
        //Swerve Modules
        //Offsets determined by manually turning all modules to 0 (forward) and recording their positions
        FrontLeftModule(12, false, -1.12),
        FrontRightModule(11, false, -1.078),
        BackLeftModule(10, false, -3.911),
        BackRightModule(9, false, -4.686);
        
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
        encoder.configFactoryDefault();
        encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        encoder.configSensorDirection(config.getReversed());
        encoder.configMagnetOffset(Units.radiansToDegrees(config.getOffset()));
        return encoder;
    }

}
