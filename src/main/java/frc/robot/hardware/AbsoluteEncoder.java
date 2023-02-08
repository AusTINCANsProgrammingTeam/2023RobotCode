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
        FrontLeftModule(1, false, -1.774, -1.12),
        FrontRightModule(2, false, -1.802, -1.078),
        BackLeftModule(3, false, 0.353, -3.911),
        BackRightModule(4, false, -0.853, -4.686),
        //Arm Encoders (REV)
        //-242, -202
        ArmBase(0, false, Units.degreesToRadians(242.8)),
        ArmElbow(1, true, Units.degreesToRadians(20.712));
        
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
        encoder.setPositionOffset(Units.radiansToRotations(Robot.isCompetitionRobot ? config.getCompetitionOffset() : config.getPracticeOffset()));
        encoder.setDistancePerRotation(config.getReversed() ? -1 : 1);
        return encoder;
    }

    public static double getPositionRadians(DutyCycleEncoder encoder){
        //Get position of a REV encoder in radians
        double position = encoder.getAbsolutePosition() - encoder.getPositionOffset();
        return Units.rotationsToRadians(encoder.getDistancePerRotation() < 0 ? 1 - position : position);
    }
}
