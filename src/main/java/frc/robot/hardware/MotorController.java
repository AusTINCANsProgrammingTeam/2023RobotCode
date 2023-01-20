package frc.robot.hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class MotorController {
    public static final class MotorDefaults{
        //Constants to use as default values for Motor Controllers
        public static final int kCurrentLimit = 40;
        public static final double kOpenLoopRampRate = 0.2;
    }

    public static enum MotorConfig {
        //Swerve Modules
        FrontLeftModuleDrive(4, 50, IdleMode.kBrake),
        FrontLeftModuleTurn(3, 40, IdleMode.kBrake, true),
        FrontRightModuleDrive(2, 50, IdleMode.kBrake),
        FrontRightModuleTurn(1, 40, IdleMode.kBrake, true),
        BackLeftModuleDrive(7, 50, IdleMode.kBrake),
        BackLeftModuleTurn(8, 40, IdleMode.kBrake, true),
        BackRightModuleDrive(6, 50, IdleMode.kBrake),
        BackRightModuleTurn(5, 40, IdleMode.kBrake, true);

        private int ID;
        private int currentLimit;
        private IdleMode idleMode;
        private double openLoopRampRate;
        private boolean reversed;

        MotorConfig(int ID, int currentLimit, IdleMode idleMode, double openLoopRampRate, boolean reversed){
            this.ID = ID;
            this.currentLimit = currentLimit;
            this.idleMode = idleMode;
            this.openLoopRampRate = openLoopRampRate;
            this.reversed = reversed;
        }

        MotorConfig(int ID, int currentLimit, IdleMode idleMode, double openLoopRampRate){
            this(ID, currentLimit, idleMode, openLoopRampRate, false);
        }

        MotorConfig(int ID, int currentLimit, IdleMode idleMode, boolean reversed){
            this(ID, currentLimit, idleMode, MotorDefaults.kOpenLoopRampRate, reversed);
        }

        MotorConfig(int ID, int currentLimit, IdleMode idleMode){
            this(ID, currentLimit, idleMode, MotorDefaults.kOpenLoopRampRate, false);
        }

        MotorConfig(int ID, int currentLimit, boolean reversed){
            this(ID, currentLimit, IdleMode.kCoast, MotorDefaults.kOpenLoopRampRate, reversed);
        }

        MotorConfig(int ID, int currentLimit){
            this(ID, currentLimit, IdleMode.kCoast, MotorDefaults.kOpenLoopRampRate, false);
        }

        MotorConfig(int ID, boolean reversed){
            this(ID, MotorDefaults.kCurrentLimit, IdleMode.kCoast, MotorDefaults.kOpenLoopRampRate, reversed);
        }

        MotorConfig(int ID){
            this(ID, MotorDefaults.kCurrentLimit, IdleMode.kCoast, MotorDefaults.kOpenLoopRampRate, false);
        }

        public int getID(){
            return ID;
        }

        public int getCurrentLimit(){
            return currentLimit;
        }

        public IdleMode getIdleMode(){
            return idleMode;
        }

        public double getOpenLoopRampRate(){
            return openLoopRampRate;
        }

        public boolean getReversed(){
            return reversed;
        }
    }

    public static CANSparkMax constructMotor(MotorConfig config){
        try {
            CANSparkMax motor = new CANSparkMax(config.getID(), MotorType.kBrushless);
            motor.restoreFactoryDefaults();
            motor.setSmartCurrentLimit(config.getCurrentLimit());
            motor.setIdleMode(config.getIdleMode());
            motor.setOpenLoopRampRate(config.getOpenLoopRampRate());
            motor.setInverted(config.getReversed());
            return motor;
        }
        catch(Exception e) {
            return null;
        }
    }
}
