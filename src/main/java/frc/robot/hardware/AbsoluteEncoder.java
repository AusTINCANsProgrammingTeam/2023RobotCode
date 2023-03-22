package frc.robot.hardware;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Robot;

public class AbsoluteEncoder {
    public enum EncoderConfig {
        //Swerve Modules (CAN)
        //Offsets determined by manually turning all modules to 0 (forward) and recording their positions
        FrontLeftModule(1, false, -4.916, -1.12),
        FrontRightModule(2, false, -4.944, -1.078),
        BackLeftModule(3, false, -2.789, -3.911),
        BackRightModule(4, false, -3.995, -4.686),
        //Arm Encoders (REV)
        ArmBase(1, true, Units.degreesToRadians(-75.45)),
        ArmChooChoo(2),
        ArmElbow(0, true, Units.degreesToRadians(218.022));
        
        private int ID;
        private boolean reversed;
        private double competitionOffset; //Offset in radians, used on competition bot
        private double practiceOffset; //Offset in radians, used on practice bot
    
        EncoderConfig(int ID, boolean reversed, double offset, double altOffset){
            this.ID = ID;
            this.reversed = reversed;
            this.competitionOffset = offset;
            this.practiceOffset = altOffset;
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

        public double getCompetitionOffset(){
            return competitionOffset;
        }

        public double getPracticeOffset(){
            return practiceOffset;
        }
    }

    public static WPI_CANCoder constructCANCoder(EncoderConfig config){
        WPI_CANCoder encoder = new WPI_CANCoder(config.getID());
        encoder.configFactoryDefault();
        encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        encoder.configSensorDirection(config.getReversed());
        encoder.configMagnetOffset(Units.radiansToDegrees(Robot.isCompetitionRobot ? config.getCompetitionOffset() : config.getPracticeOffset()));
        return encoder;
    }

    public static DutyCycleEncoder constructREVEncoder(EncoderConfig config){
        DutyCycleEncoder encoder = new DutyCycleEncoder(config.getID());
        double offset = Units.radiansToRotations(Robot.isCompetitionRobot ? config.getCompetitionOffset() : config.getPracticeOffset());
        if (offset < 0){
            offset++;
        }
        encoder.setPositionOffset(offset);
        encoder.setDistancePerRotation(config.getReversed() ? -1 : 1);
        return encoder;
    }

    public static double getPositionRadians(DutyCycleEncoder encoder){
        //Get position of a REV encoder in radians
        double position = encoder.getAbsolutePosition() + encoder.getPositionOffset();
        position = encoder.getDistancePerRotation() < 0 ? 1 - position : position;
        if(position < 0){
            position++;
        }
        return Units.rotationsToRadians(position);
    }

    public static double getPositionRadians(DutyCycleEncoder encoder, int places){
        //Truncates measure to places decimal points
        return Math.round(getPositionRadians(encoder) * Math.pow(10, places)) / Math.pow(10, places);
    }
}
