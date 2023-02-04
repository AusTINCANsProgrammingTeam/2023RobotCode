package frc.robot.hardware;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.util.Units;
import frc.robot.Robot;

public class AbsoluteEncoder {
    public enum EncoderConfig {
        //Swerve Modules
        //Offsets determined by manually turning all modules to 0 (forward) and recording their positions
        FrontLeftModule(1, false, -1.774, -1.12),
        FrontRightModule(2, false, -1.802),
        BackLeftModule(3, false, 0.353),
        BackRightModule(4, false, -0.853);
        
        private int ID;
        private boolean reversed;
        private double offset; //Offset in radians
        private double altOffset; //Offset in radians, used on practice bot
    
        EncoderConfig(int ID, boolean reversed, double offset, double altOffset){
            this.ID = ID;
            this.reversed = reversed;
            this.offset = offset;
            this.altOffset = altOffset;
        }

        EncoderConfig(int ID, boolean reversed, double offset){
            this(ID, reversed, offset, 0);
        }
    
        EncoderConfig(int ID, boolean reversed){
            this(ID, reversed, 0, 0);
        }
    
        EncoderConfig(int ID){
            this(ID, false, 0, 0);
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

        public double getAltOffset(){
            return altOffset;
        }
    }

    public static WPI_CANCoder constructEncoder(EncoderConfig config){
        WPI_CANCoder encoder = new WPI_CANCoder(config.getID());
        encoder.configFactoryDefault();
        encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        encoder.configSensorDirection(config.getReversed());
        encoder.configMagnetOffset(Units.radiansToDegrees(Robot.isCompetitionRobot ? config.getOffset() : config.getAltOffset()));
        return encoder;
    }
}
