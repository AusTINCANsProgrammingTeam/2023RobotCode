package frc.robot.hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.MotorDefaults;

public class MotorController {
    public static enum MotorConfig {
        //Swerve Modules
        FrontLeftModuleDrive(4, 50),
        FrontLeftModuleTurn(3),
        FrontRightModuleDrive(2, 50),
        FrontRightModuleTurn(1),
        BackLeftModuleDrive(7, 50),
        BackLeftModuleTurn(8),
        BackRightModuleDrive(6, 50),
        BackRightModuleTurn(5);

        private int ID;
        private int currentLimit;
        private double openLoopRampRate;
        private boolean reversed;

        MotorConfig(int ID, int currentLimit, double openLoopRampRate, boolean reversed){
            this.ID = ID;
            this.currentLimit = currentLimit;
            this.openLoopRampRate = openLoopRampRate;
            this.reversed = reversed;
        }

        MotorConfig(int ID, int currentLimit, double openLoopRampRate){
            this(ID, currentLimit, openLoopRampRate, false);
        }

        MotorConfig(int ID, int currentLimit, boolean reversed){
            this(ID, currentLimit, MotorDefaults.kOpenLoopRampRate, reversed);
        }

        MotorConfig(int ID, int currentLimit){
            this(ID, currentLimit, MotorDefaults.kOpenLoopRampRate, false);
        }

        MotorConfig(int ID, boolean reversed){
            this(ID, MotorDefaults.kCurrentLimit, MotorDefaults.kOpenLoopRampRate, reversed);
        }

        MotorConfig(int ID){
            this(ID, MotorDefaults.kCurrentLimit, MotorDefaults.kOpenLoopRampRate, false);
        }

        public int getID(){
            return ID;
        }

        public int getCurrentLimit(){
            return currentLimit;
        }

        public double getOpenLoopRampRate(){
            return openLoopRampRate;
        }

        public boolean getReversed(){
            return reversed;
        }
    }

    public static CANSparkMax constructMotor(MotorConfig config){
        CANSparkMax motor = new CANSparkMax(config.getID(), MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(config.getCurrentLimit());
        motor.setOpenLoopRampRate(config.getOpenLoopRampRate());
        motor.setInverted(config.getReversed());
        return motor;
    }
}
