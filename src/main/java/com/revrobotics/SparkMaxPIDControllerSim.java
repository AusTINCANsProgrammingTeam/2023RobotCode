// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Spoofing the package so that the SparkMaxPIDController constructor is visible
// to make inheritance possible
package com.revrobotics;

import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.RelativeEncoderSim;

/** Add your docs here. */
public class SparkMaxPIDControllerSim extends SubsystemBase {

    RelativeEncoderSim pos;
    double setPoint = 0;

    double error, lastError, sumError = 0;
    CANSparkMax max;
    SparkMaxPIDController ctrl;

    /**
     * Constructs a SparkMaxPIDControllerSim from a CANSparkMax object
     * and the RelativeEncoderSim that tracks the motor's position in simulation
     *
     * @return Current relative position of the motor in Rotations 
    */
    public SparkMaxPIDControllerSim(CANSparkMax spark, RelativeEncoderSim encoder) {
        ctrl = spark.getPIDController();
        pos = encoder;
        max = spark;
    }

    /**
     * Sets the set point for the PID loop target.
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
        max.set(ctrl.getFF() + ctrl.getP()*error + ctrl.getI()*sumError + ctrl.getD()*(error - lastError));

    }
}
