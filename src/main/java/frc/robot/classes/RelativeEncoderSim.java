// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;

public class RelativeEncoderSim implements AutoCloseable {
    private RelativeEncoder encoder;
    private SimDevice simDevice;
    private SimDouble simVelocity;

    public RelativeEncoderSim(CANSparkMax sparkMax) {
        this.encoder = sparkMax.getEncoder();
        simDevice = SimDevice.create("RelEncoderSim", sparkMax.getDeviceId());
        if (simDevice != null) { // Only null if SimDevice name is not unique
            simVelocity = simDevice.createDouble("Velocity", SimDevice.Direction.kInput, 0.0);
        }
    }
    
    /**
     * Direct call to RelativeEncoder getPosition method since it works in simulation as is.
     * Included in this class for convienience
     *
     * @return Current relative position of the motor in Rotations 
     */
    public double getPosition() {
        return encoder.getPosition();
    };

    /**
     * Get the simulated velocity of the motor. 
     * 
     * User should keep the velocity conversion factor setting in mind, 
     * but it is not factored into the stored value.
     *
     * @return Number the RPM of the motor
     */
    public double getVelocity() {
        return simVelocity.get();
    };

    /**
     * Direct call to RelativeEncoder setPosition method since it works in simulation as is.
     * Included in this class for convienience
     *
     * @param position Number of rotations of the motor
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setPosition(double position) {
        return encoder.setPosition(position);
    };

    /**
     * Set the simulated velocity of the motor. 
     * 
     * User should keep the velocity conversion factor setting in mind, 
     * but it is not factored into the stored value.
     *
     * @param velocity RPMs of the motor to set
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setSimVelocity(double velocity) {
        simVelocity.set(velocity);
        return REVLibError.kOk;
    }

    @Override
    public void close() throws Exception {
        simDevice.close();
    }

}