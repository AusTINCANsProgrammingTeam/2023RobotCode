// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class SparkMaxPIDControllerSim extends SubsystemBase {

    RelativeEncoderSim pos;
    double setPoint = 0;

    double error, lastError, sumError = 0;
    CANSparkMax motorCtrl;
    SparkMaxPIDController pidCtrl;

    /**
     * Constructs a SparkMaxPIDControllerSim from a CANSparkMax object
     * and the RelativeEncoderSim that tracks the motor's position in simulation
     *
     * @return Current relative position of the motor in Rotations 
    */
    public SparkMaxPIDControllerSim(CANSparkMax spark, RelativeEncoderSim encoder) {
        pidCtrl = spark.getPIDController();
        pos = encoder;
        motorCtrl = spark;
    }

    /**
     * Sets the setpoint for the PID loop target.
     * Only supports position ControlType.
     *
     * @return kNotImplemented if ControlType is not kPosition, kOK otherwise
    */
    public REVLibError setReference(double value, ControlType ctrl) {
        if (ctrl != ControlType.kPosition) {
            DriverStation.reportError("SparkMaxPIDControllerSim ControlType not supported", true);
            return REVLibError.kNotImplemented;
        }
        setPoint = value;
        return REVLibError.kOk;

    }


    @Override
    public void simulationPeriodic() {
        // Update error values
        lastError = error;
        error = setPoint - pos.getPosition();
        sumError += error;

        // Calculate next motor setting
        motorCtrl.set(pidCtrl.getFF() + pidCtrl.getP()*error + pidCtrl.getI()*sumError + pidCtrl.getD()*(error - lastError));

    }
}
