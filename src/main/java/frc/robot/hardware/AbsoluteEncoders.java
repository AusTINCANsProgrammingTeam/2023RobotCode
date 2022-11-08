package frc.robot.hardware;

import edu.wpi.first.wpilibj.RobotController;

public enum AbsoluteEncoders {
    //Swerve Modules
    FrontLeftModule(9),
    FrontRightModule(10),
    BackLeftModule(11),
    BackRightModule(12);
    
    private int ID;
    private boolean reversed;
    private double offset; //Offset in radians

    AbsoluteEncoders(int ID, boolean reversed, double offset){
        this.ID = ID;
        this.reversed = reversed;
        this.offset = offset;
    }

    AbsoluteEncoders(int ID, boolean reversed){
        this(ID, reversed, 0);
    }

    AbsoluteEncoders(int ID){
        this(ID, false, 0);
    }

    public double convertVoltageToRadians(double voltage){ 
        //Calculate the value of the absolute encoder in radians
        double angle = voltage / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= offset;
        return angle * (reversed ? -1.0 : 1.0);
    }

    public int getID(){
        return ID;
    }
}
