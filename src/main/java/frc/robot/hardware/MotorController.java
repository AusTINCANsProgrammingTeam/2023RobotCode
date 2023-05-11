package frc.robot.hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;

public class MotorController {
    public static final class MotorDefaults{
        //Constants to use as default values for Motor Controllers
        public static final int kCurrentLimit = 40;
        public static final double kOpenLoopRampRate = 0.2;
    }

    public static enum MotorConfig {
        //Swerve Modules
        FrontLeftModuleDrive(20, 50, IdleMode.kBrake),
        FrontLeftModuleTurn(19, 40, IdleMode.kBrake, true),
        FrontRightModuleDrive(2, 50, IdleMode.kBrake),
        FrontRightModuleTurn(1, 40, IdleMode.kBrake, true),
        BackLeftModuleDrive(11, 50, IdleMode.kBrake),
        BackLeftModuleTurn(12, 40, IdleMode.kBrake, true),
        BackRightModuleDrive(10, 50, IdleMode.kBrake),
        BackRightModuleTurn(9, 40, IdleMode.kBrake, true),
        //Arm motors
        ArmBase1(14, 50, IdleMode.kBrake),
        ArmBase2(13, 50, IdleMode.kBrake),
        ArmElbow(15, 50, true),  
        //Intake motors
        IntakeMotor1(4, 20, IdleMode.kBrake),
        IntakeMotor2(5, 20, IdleMode.kBrake),
        //BuddyBalance Motors
        BuddyBalanceRight(7, 40, IdleMode.kBrake),
        BuddyBalanceLeft(8, 40, IdleMode.kBrake, true); // TODO: update IDs for buddy balance motors when robot is finalized

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
        CANSparkMax motor = new CANSparkMax(config.getID(), MotorType.kBrushless);
        int errorCode = motor.restoreFactoryDefaults().value;
        errorCode += motor.setSmartCurrentLimit(config.getCurrentLimit()).value;
        errorCode += motor.setIdleMode(config.getIdleMode()).value;
        errorCode +=  motor.setOpenLoopRampRate(config.getOpenLoopRampRate()).value;
        motor.setInverted(config.getReversed());
        if(errorCode != 0){DriverStation.reportError("An Error has occured in constructMotor(MotorConfig config) with config " + config.getID(), null);}
        return motor;
    }

    public static CANSparkMax constructMotor(MotorConfig config, double[] PIDArray){
        CANSparkMax motor = constructMotor(config);
        SparkMaxPIDController motorPIDcontroller = motor.getPIDController();
        int errorCode = motorPIDcontroller.setP(PIDArray[0]).value; //Gets REVLibError code value (Is 0 if no error occured)
        errorCode += motorPIDcontroller.setI(PIDArray[1]).value;
        errorCode += motorPIDcontroller.setD(PIDArray[2]).value;
        if(errorCode != 0){DriverStation.reportError("An Error has occured in constructMotor(MotorConfig config, double[] PIDArray) with motor " + motor.getDeviceId() + " whilst setting PID", null);}
        return motor;
    }

    public static void errorCheck(REVLibError error) {
        if(error != REVLibError.kOk){DriverStation.reportError("An error has occured! ", true);}
    }
}
