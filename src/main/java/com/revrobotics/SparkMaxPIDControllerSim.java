// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Spoofing the package so that the SparkMaxPIDController constructor is visible
// to make inheritance possible
package com.revrobotics;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.classes.RelativeEncoderSim;

/** Add your docs here. */
public class SparkMaxPIDControllerSim extends SparkMaxPIDController {

    double p,i,d,ff = 0;
    RelativeEncoderSim pos;
    double setPoint = 0;

    double error, lastError, sumError = 0;
    CANSparkMax max;

    public SparkMaxPIDControllerSim(CANSparkMax spark, RelativeEncoderSim encoder) {
        super(spark);
        pos = encoder;
        max = spark;
    }

    public REVLibError setP(double gain) {
        p = gain;
        return REVLibError.kOk;
    }

    public REVLibError setReference(double value, ControlType ctrl) {
        if (ctrl != ControlType.kPosition) {
            DriverStation.reportError("SparkMaxPIDControllerSim ControlType not supported", true);
            return REVLibError.kNotImplemented;
        }
        setPoint = value;
        return REVLibError.kOk;

    }


    // Actually runs the PIDF and updates the motor. Needed innocuous method to override for simulation to work with inheritance
    public double getP() {
        lastError = error;
        error = setPoint - pos.getPosition();
        sumError += error;
        max.set(ff + p*error + i*sumError + d*(error - lastError));
        return p;


    }
}
