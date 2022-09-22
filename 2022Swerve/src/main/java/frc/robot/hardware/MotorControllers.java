package frc.robot.hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.MotorDefaults;

public enum MotorControllers {
    //Swerve Modules
    FrontLeftModuleDrive(1, 50),
    FrontLeftModuleTurn(2),
    FrontRightModuleDrive(3, 50),
    FrontRightModuleTurn(4),
    BackLeftModuleDrive(5, 50),
    BackLeftModuleTurn(6),
    BackRightModuleDrive(7, 50),
    BackRightModuleTurn(8);

    private int ID;
    private int currentLimit;
    private double openLoopRampRate;
    private boolean reversed;
    private CANSparkMax motor;

    MotorControllers(int ID, int currentLimit, double openLoopRampRate, boolean reversed){
        this.ID = ID;
        this.currentLimit = currentLimit;
        this.openLoopRampRate = openLoopRampRate;
        this.reversed = reversed;
        initializeMotor();
    }

    MotorControllers(int ID, int currentLimit, double openLoopRampRate){
        this.ID = ID;
        this.currentLimit = currentLimit;
        this.openLoopRampRate = openLoopRampRate;
        this.reversed = false;
        initializeMotor();
    }

    MotorControllers(int ID, int currentLimit, boolean reversed){
        this.ID = ID;
        this.currentLimit = currentLimit;
        this.openLoopRampRate = MotorDefaults.kOpenLoopRampRate;
        this.reversed = reversed;
        initializeMotor();
    }

    MotorControllers(int ID, int currentLimit){
        this.ID = ID;
        this.currentLimit = currentLimit;
        this.openLoopRampRate = MotorDefaults.kOpenLoopRampRate;
        this.reversed = false;
        initializeMotor();
    }

    MotorControllers(int ID, boolean reversed){
        this.ID = ID;
        this.currentLimit = MotorDefaults.kCurrentLimit;
        this.openLoopRampRate = MotorDefaults.kOpenLoopRampRate;
        this.reversed = reversed;
        initializeMotor();
    }

    MotorControllers(int ID){
        this.ID = ID;
        this.currentLimit = MotorDefaults.kCurrentLimit;
        this.openLoopRampRate = MotorDefaults.kOpenLoopRampRate;
        this.reversed = false;
        initializeMotor();
    }

    private void initializeMotor(){
        motor = new CANSparkMax(ID, MotorType.kBrushless);
        motor.setOpenLoopRampRate(openLoopRampRate);
        motor.setSmartCurrentLimit(currentLimit);
        motor.setInverted(reversed);
    }

    public CANSparkMax getMotor(){
        return motor;
    }
}
